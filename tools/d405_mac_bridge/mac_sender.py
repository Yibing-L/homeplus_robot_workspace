#!/usr/bin/env python3
"""Capture the D405 natively on macOS and stream it to the ROS 2 VM.

UTM USB passthrough cannot carry RealSense depth reliably, so the camera stays on
the Mac. This streams color (rgb8) + depth aligned to color (uint16, millimetres,
same convention the pipeline assumes for realsense-ros 16UC1) + color intrinsics
over TCP. vm_receiver.py republishes them on the /cameras/d405/... topics.

Run with root (librealsense needs it on macOS):
    sudo .venv/bin/python mac_sender.py [--fps 15] [--bind 192.168.64.1] [--port 8766]
"""

import argparse
import json
import socket
import struct
import time

import numpy as np
import pyrealsense2 as rs


def intrinsics_dict(intr):
    return {
        'width': intr.width, 'height': intr.height,
        'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy,
        'model': str(intr.model), 'coeffs': list(intr.coeffs),
    }


def send_msg(conn, header, *payloads):
    head = json.dumps(header).encode()
    conn.sendall(struct.pack('!I', len(head)) + head)
    for p in payloads:
        conn.sendall(p)


def start_pipeline(args):
    pipeline = rs.pipeline()
    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipeline.start(cfg)
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    color_prof = profile.get_stream(rs.stream.color).as_video_stream_profile()
    depth_prof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    ext = depth_prof.get_extrinsics_to(color_prof)
    hello = {
        'type': 'hello',
        'device': dev.get_info(rs.camera_info.name),
        'serial': dev.get_info(rs.camera_info.serial_number),
        'firmware': dev.get_info(rs.camera_info.firmware_version),
        'usb': dev.get_info(rs.camera_info.usb_type_descriptor),
        'fps': args.fps,
        'depth_scale': depth_scale,
        'color_intrinsics': intrinsics_dict(color_prof.get_intrinsics()),
        'depth_intrinsics': intrinsics_dict(depth_prof.get_intrinsics()),
        # p_color = R * p_depth + t, R column-major as librealsense reports it
        'depth_to_color': {'rotation': list(ext.rotation), 'translation': list(ext.translation)},
    }
    print(f"[d405] {hello['device']} serial={hello['serial']} fw={hello['firmware']} "
          f"usb={hello['usb']} depth_scale={depth_scale}", flush=True)
    return pipeline, hello, depth_scale


def serve(args):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)
    # Start the camera once: stopping/restarting a pipeline segfaults pyrealsense2 on macOS.
    pipeline, hello, depth_scale = start_pipeline(args)
    align = rs.align(rs.stream.color)
    # Intel's recommended post-processing: smooth in disparity space, then average over
    # frames. At ~1 m (beyond the D405's 7-50 cm sweet spot) raw depth on the table
    # jitters by several cm per frame, which MoveIt's OctoMap turns into phantom
    # obstacles just above the table. The temporal filter's delta threshold resets
    # pixels whose depth jumps, so a moving camera is not smeared.
    depth_filters = [] if args.no_depth_filter else [
        rs.disparity_transform(True), rs.spatial_filter(), rs.temporal_filter(),
        rs.disparity_transform(False)]
    print(f"[d405] depth filters: {'off (raw)' if args.no_depth_filter else 'spatial + temporal'}", flush=True)
    print(f'[d405] waiting for VM receiver on {args.bind}:{args.port}', flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f'[d405] receiver connected from {addr}', flush=True)
        try:
            send_msg(conn, hello)
            to_mm = depth_scale * 1000.0
            seq, t_report, n_report = 0, time.time(), 0
            while True:
                frames = pipeline.wait_for_frames(5000)
                frames = align.process(frames)
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if not color or not depth:
                    continue
                # Filter the depth frame only; colour never passes through a filter.
                for f in depth_filters:
                    depth = f.process(depth)
                depth = depth.as_depth_frame()
                c = np.asanyarray(color.get_data())
                d = np.asanyarray(depth.get_data())
                if abs(to_mm - 1.0) > 1e-9:
                    d = np.clip(np.rint(d.astype(np.float32) * to_mm), 0, 65535).astype(np.uint16)
                cb, db = c.tobytes(), np.ascontiguousarray(d, dtype='<u2').tobytes()
                send_msg(conn, {
                    'type': 'frame', 'seq': seq, 'mac_time': time.time(),
                    'hw_ts_ms': color.get_timestamp(),
                    'w': c.shape[1], 'h': c.shape[0], 'color_len': len(cb), 'depth_len': len(db),
                }, cb, db)
                seq += 1
                n_report += 1
                if time.time() - t_report >= 5.0:
                    print(f'[d405] sent {n_report / (time.time() - t_report):.1f} fps (seq {seq})', flush=True)
                    t_report, n_report = time.time(), 0
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f'[d405] receiver disconnected: {e}', flush=True)
        except RuntimeError as e:
            print(f'[d405] camera error: {e}', flush=True)
            time.sleep(1.0)
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bind', default='192.168.64.1', help='Mac address on the UTM shared network')
    ap.add_argument('--no-depth-filter', action='store_true',
                    help='send raw depth (no spatial/temporal post-processing)')
    ap.add_argument('--port', type=int, default=8766)
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--serial', default='')
    serve(ap.parse_args())


if __name__ == '__main__':
    main()
