"""Offline tests for the detector's remote-inference client and diagnostics.

Runs the real MolmoPointNode.detect_loop through tests/detector_replay.py
against a fake CONTRACT.md server; needs neither ROS nor a GPU.
"""
import base64
import importlib.util
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location(
    'detector_replay', Path(__file__).with_name('detector_replay.py'))
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)

CUP_UV, CUP_RADIUS_PX = (320, 260), 50


def synthetic_cup_frame():
    """Camera 1 m above the floor looking along world +x at a cup 0.5 m away."""
    color = np.full((480, 640, 3), 90, np.uint8)
    cv2.circle(color, CUP_UV, CUP_RADIUS_PX, (0, 0, 255), -1)
    depth = np.full((480, 640), 900, np.uint16)
    disk = np.zeros((480, 640), np.uint8)
    cv2.circle(disk, CUP_UV, CUP_RADIUS_PX, 1, -1)
    depth[disk > 0] = 500
    world_from_camera = np.eye(4)
    # Optical x -> world -y, optical y -> world -z, optical z -> world +x.
    world_from_camera[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    world_from_camera[:3, 3] = [0, 0, 1.0]
    return {
        'color': color, 'depth': depth, 'depth_encoding': '16UC1',
        'K': np.array([600, 0, 320, 0, 600, 240, 0, 0, 1], float),
        'camera_width': 640, 'camera_height': 480,
        'T_world_camera': world_from_camera, 'stamp_ns': 1_000_000_000_000,
        'camera_frame': 'd405_color_optical_frame', 'world_frame': 'world',
    }


class FakeServer:
    """CONTRACT.md endpoints answering with the synthetic cup."""

    def __init__(self):
        self.points = [[0, 0, float(CUP_UV[0]), float(CUP_UV[1])]]
        self.return_mask = True
        self.requests = []
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def _reply(self, payload):
                body = json.dumps(payload).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                self._reply({'ok': True, 'molmopoint': True, 'sam2': server.return_mask,
                             'device': 'fake', 'quantization': '4bit'})

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                image = cv2.imdecode(np.frombuffer(
                    base64.b64decode(body['image_jpeg_b64']), np.uint8), cv2.IMREAD_COLOR)
                server.requests.append((self.path, body, image.shape))
                if self.path == '/point':
                    self._reply({'prompt': f"Point to {body['caption']}",
                                 'generated_text': 'fake', 'points': server.points,
                                 'image_size': [image.shape[1], image.shape[0]],
                                 'elapsed_sec': 0.0})
                else:
                    mask = None
                    if server.return_mask:
                        disk = np.zeros(image.shape[:2], np.uint8)
                        cv2.circle(disk, CUP_UV, CUP_RADIUS_PX, 255, -1)
                        mask = base64.b64encode(cv2.imencode('.png', disk)[1]).decode()
                    self._reply({'mask_png_b64': mask, 'score': 0.9 if mask else None,
                                 'elapsed_sec': 0.0})

        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.httpd.server_address[1]}'
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


class RemoteDetectorTests(unittest.TestCase):
    def setUp(self):
        self.server = FakeServer()
        self.addCleanup(self.server.close)
        self.frame = synthetic_cup_frame()

    def run_cycle(self, **kwargs):
        kwargs.setdefault('server_url', self.server.url)
        return replay.run_replay(self.frame, echo_logs=False, **kwargs)

    def test_remote_pipeline_publishes_cup_pose(self):
        result = self.run_cycle()
        diag = result['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']), ('done', 'pose_published'))
        self.assertEqual(diag['backend'], 'remote')
        self.assertTrue(diag['tf_ok'])
        self.assertEqual(diag['n_model_points'], 1)
        self.assertGreater(diag['mask_pixels'], 7000)
        self.assertAlmostEqual(diag['near_depth_m'], 0.5, places=3)
        # Cup surface 0.5 m ahead; v=260 is 20 px below center at f=600.
        np.testing.assert_allclose(result['pose_world'], [0.5, 0.0, 1 - 20 * 0.5 / 600],
                                   atol=0.003)
        self.assertEqual([r[0] for r in self.server.requests], ['/point', '/segment'])
        self.assertEqual(self.server.requests[0][2], (480, 640, 3))
        self.assertEqual(self.server.requests[0][1]['caption'], 'red cup')
        self.assertEqual(self.server.requests[1][1]['point'], list(CUP_UV))
        self.assertIsNotNone(result['debug_image'])

    def test_never_still_motion_status_is_named(self):
        diag = self.run_cycle(motion_status=False)['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']),
                         ('motion_gate', 'motion_status_never_received'))
        self.assertTrue(diag['robot_moving'])
        self.assertEqual(self.server.requests, [])

    def test_rgbd_skew_is_named(self):
        self.frame['depth_stamp_ns'] = self.frame['stamp_ns'] + 100_000_000
        diag = self.run_cycle()['diagnostics']
        self.assertEqual(diag['reason'], 'rgbd_skew')
        self.assertAlmostEqual(diag['rgbd_skew_sec'], 0.1)

    def test_tf_failure_is_named(self):
        self.frame['T_world_camera'] = None
        diag = self.run_cycle()['diagnostics']
        self.assertEqual((diag['stage'], diag['reason'], diag['tf_ok']),
                         ('tf', 'tf_lookup_failed', False))

    def test_stale_frame_discarded_after_inference_with_debug_image(self):
        result = self.run_cycle(frame_age_sec=20.0)
        diag = result['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']), ('publish_gate', 'frame_stale'))
        self.assertIsNotNone(result['debug_image'])
        self.assertIsNone(result['pose_world'])

    def test_missing_mask_is_named(self):
        self.server.return_mask = False
        result = self.run_cycle()
        diag = result['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']), ('geometry', 'sam2_returned_no_mask'))
        self.assertEqual(diag['mask_pixels'], 0)
        self.assertIsNotNone(result['debug_image'])

    def test_insufficient_mask_depth_reports_counts(self):
        self.frame['depth'][self.frame['depth'] == 500] = 0
        diag = self.run_cycle()['diagnostics']
        self.assertEqual(diag['stage'], 'geometry')
        self.assertIn('too few valid depth pixels inside mask: 0', diag['reason'])
        self.assertEqual(diag['mask_depth_valid'], 0)

    def test_surface_offset_rejection_is_named(self):
        diag = self.run_cycle(overrides={'cup_surface_to_center_m': 0.2})['diagnostics']
        self.assertEqual(diag['stage'], 'geometry')
        self.assertIn('offset_surface_goal', diag['reason'])
        self.assertIn('surface_world', diag)

    def test_no_points_is_named(self):
        self.server.points = [[0, 1, 10.0, 10.0]]  # Only image_index 1: filtered.
        diag = self.run_cycle()['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']), ('inference', 'no_model_points'))
        self.assertEqual((diag['raw_points'], diag['image0_points']), (1, 0))

    def test_unreachable_server_is_named(self):
        self.server.close()
        diag = self.run_cycle()['diagnostics']
        self.assertEqual((diag['stage'], diag['reason']), ('inference', 'inference_failed'))

    def test_manual_point_and_mask_without_server(self):
        mask = np.zeros((480, 640), np.uint8)
        cv2.circle(mask, CUP_UV, CUP_RADIUS_PX, 255, -1)
        result = replay.run_replay(self.frame, point=CUP_UV, mask=mask, echo_logs=False)
        self.assertEqual(result['diagnostics']['reason'], 'pose_published')


class PointSelectionParityTests(unittest.TestCase):
    """Remote and local inference share candidate filtering and selection."""

    def test_remote_and_local_points_give_identical_detections(self):
        state = {'logs': [], 'published': {}, 'echo_logs': False}
        module = replay.load_detector(state)
        node = module.MolmoPointNode.__new__(module.MolmoPointNode)
        node.get_logger = lambda: replay._Logger(state)
        node.max_points, node.point_selection = 1, 'closest_to_center'
        raw = [[0, 0, 100.4, 50.6], [1, 0, 330.0, 250.0], [2, 1, 320.0, 240.0]]
        node._local_point = lambda image, caption: ('Point to cup', 'txt', raw)
        node._remote_point = lambda image, caption: ('Point to cup', 'txt', raw)
        image = np.zeros((480, 640, 3), np.uint8)
        node.remote_inference_url = ''
        local = node.run_model_inference(image, 'cup')
        node.remote_inference_url = 'http://x'
        remote = node.run_model_inference(image, 'cup')
        self.assertEqual(local, remote)
        self.assertEqual(local, [{
            'label': 'cup', 'score': 1.0, 'backend': 'molmopoint', 'object_id': 1,
            'point': [330, 250], 'bbox': [318, 238, 342, 262]}])


if __name__ == '__main__':
    unittest.main()
