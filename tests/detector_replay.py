#!/usr/bin/env python3
"""Replay one saved RGB-D frame through the real MolmoPoint detector cycle.

Runs ``MolmoPointNode.detect_loop`` from homeplus_vision/scripts/
grounding_dino_node.py unchanged (motion gate, TF, inference, SAM2 mask,
mask-depth geometry, publish gate) against lightweight ROS stand-ins, so it
works on a laptop without ROS, cameras or a GPU. Model calls go to a remote
inference server (parallel_prompts/CONTRACT.md), or can be replaced with a
manual point/mask to debug geometry alone.

Inputs
  *.npz   written by the node's ``debug_dump_dir`` parameter, or by hand with
          keys: color (HxWx3 BGR uint8), depth (HxW uint16 mm or float32 m),
          K (9 floats), camera_width, camera_height; optional: T_world_camera
          (4x4), stamp_ns, depth_stamp_ns, depth_encoding, caption, task_id.
  bag dir a ros2 bag (needs a sourced ROS 2 environment) containing the color,
          aligned depth and camera_info topics from the node config, plus
          /tf and /tf_static for the acquisition-time camera pose.

Examples
  python3 tests/detector_replay.py cycle.npz --server http://192.168.64.1:8765
  python3 tests/detector_replay.py my_bag --bag-index 30 --server http://... \\
      --to-npz frame.npz --debug-out debug.png
  python3 tests/detector_replay.py cycle.npz --point 320 250 --mask mask.png
  python3 tests/detector_replay.py cycle.npz --server ... --no-motion-status
"""
import argparse
import importlib.util
import json
import math
import sys
import tempfile
import types
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = ROOT / 'homeplus_vision/scripts/grounding_dino_node.py'
DEFAULT_PARAMS = ROOT / 'homeplus_vision/config/target_detection_params.yaml'


# ----------------------------------------------------------------------------
# Minimal ROS stand-ins: just enough of rclpy/messages/tf2/cv_bridge for the
# detector module to import and run one cycle in-process.

class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = int(sec), int(nanosec)


class _Header:
    def __init__(self, stamp=None, frame_id=''):
        self.stamp = stamp or _Stamp()
        self.frame_id = frame_id


class _Time:
    def __init__(self, seconds=0, nanoseconds=0, clock_type=None):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        return _Stamp(self.nanoseconds // 1_000_000_000, self.nanoseconds % 1_000_000_000)

    @classmethod
    def from_msg(cls, msg, clock_type=None):
        return cls(nanoseconds=int(msg.sec) * 1_000_000_000 + int(msg.nanosec))


class _Duration(_Time):
    pass


class _Image:
    def __init__(self, header=None):
        self.header = header or _Header()
        self.height = self.width = 0
        self.encoding = ''
        self.array = None


class _CameraInfo:
    def __init__(self):
        self.header = _Header()
        self.width = self.height = 0
        self.k = [0.0] * 9


def _xyz(**kw):
    return SimpleNamespace(x=0.0, y=0.0, z=0.0, **kw)


class _Pose:
    def __init__(self):
        self.position = _xyz()
        self.orientation = _xyz(w=1.0)


class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class _TransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ''
        self.transform = SimpleNamespace(translation=_xyz(), rotation=_xyz(w=1.0))


class _Data:
    def __init__(self, data=None):
        self.data = data


class _Bridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        array = msg.array
        if desired_encoding == 'bgr8' and msg.encoding == 'rgb8':
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        return array

    def cv2_to_imgmsg(self, array, encoding='passthrough'):
        msg = _Image()
        msg.array = np.asarray(array)
        msg.height, msg.width = msg.array.shape[:2]
        msg.encoding = encoding
        return msg


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _quat_multiply(a, b):
    return (a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
            a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
            a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
            a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2])


def _do_transform_pose_stamped(pose, tf):
    r = tf.transform.rotation
    t = tf.transform.translation
    p = pose.pose.position
    world = _quat_matrix(r) @ [p.x, p.y, p.z] + [t.x, t.y, t.z]
    o = pose.pose.orientation
    q = _quat_multiply((r.x, r.y, r.z, r.w), (o.x, o.y, o.z, o.w))
    out = _PoseStamped()
    out.header = _Header(pose.header.stamp, tf.header.frame_id)
    out.pose.position = _xyz()
    out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, world)
    out.pose.orientation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
    return out


class _Logger:
    def __init__(self, state):
        self.state = state

    def _log(self, level, message, **_kwargs):
        self.state['logs'].append((level, message))
        if self.state.get('echo_logs', True):
            print(f'  [{level.upper():5}] {message}')

    def debug(self, message, **kw): self._log('debug', message, **kw)
    def info(self, message, **kw): self._log('info', message, **kw)
    def warn(self, message, **kw): self._log('warn', message, **kw)
    def warning(self, message, **kw): self._log('warn', message, **kw)
    def error(self, message, **kw): self._log('error', message, **kw)


class _Publisher:
    def __init__(self, state, topic):
        self.state, self.topic = state, topic

    def publish(self, msg):
        self.state['published'].setdefault(self.topic, []).append(msg)


def _make_node_class(state):
    class Node:
        def __init__(self, name, **_kwargs):
            self._params = {}
            self._logger = _Logger(state)
            self._clock = SimpleNamespace(
                now=lambda: _Time(nanoseconds=state['now_ns']))

        def declare_parameter(self, name, default=None):
            value = state['overrides'].get(name, default)
            self._params[name] = SimpleNamespace(value=value)

        def get_parameter(self, name):
            return self._params[name]

        def create_publisher(self, _type, topic, _qos):
            return _Publisher(state, topic)

        def create_subscription(self, *_args, **_kwargs):
            return None

        def create_timer(self, *_args, **_kwargs):
            return None

        def add_on_set_parameters_callback(self, _callback):
            return None

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return self._clock

    return Node


class _TfBuffer:
    def __init__(self, state):
        self.state = state

    def lookup_transform(self, target, source, _time, timeout=None):
        if self.state.get('tf') is None:
            raise RuntimeError(self.state.get('tf_error') or
                               f'"{target}" -> "{source}" not available (replay)')
        return self.state['tf']


def _stub_modules(state):
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        return mod

    noop_class = type('Noop', (), {'__init__': lambda self, *a, **k: None})
    return {
        'rclpy': module('rclpy', init=lambda **k: None, ok=lambda: False,
                        shutdown=lambda: None),
        'rclpy.node': module('rclpy.node', Node=_make_node_class(state)),
        'rclpy.duration': module('rclpy.duration', Duration=_Duration),
        'rclpy.time': module('rclpy.time', Time=_Time),
        'rclpy.callback_groups': module(
            'rclpy.callback_groups', MutuallyExclusiveCallbackGroup=noop_class,
            ReentrantCallbackGroup=noop_class),
        'rclpy.executors': module('rclpy.executors', MultiThreadedExecutor=noop_class),
        'rclpy.qos': module(
            'rclpy.qos', qos_profile_sensor_data=None, QoSProfile=noop_class,
            DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)),
        'ament_index_python': module('ament_index_python'),
        'ament_index_python.packages': module(
            'ament_index_python.packages',
            get_package_share_directory=lambda _pkg: str(ROOT / 'homeplus_vision')),
        'rcl_interfaces': module('rcl_interfaces'),
        'rcl_interfaces.msg': module(
            'rcl_interfaces.msg',
            SetParametersResult=lambda successful=True, reason='': SimpleNamespace(
                successful=successful, reason=reason)),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', Image=_Image, CameraInfo=_CameraInfo),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=_PoseStamped,
                                    TransformStamped=_TransformStamped),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module('std_msgs.msg', Bool=_Data, String=_Data, Header=_Header),
        'tf2_ros': module('tf2_ros', Buffer=lambda **k: _TfBuffer(state),
                          TransformListener=noop_class),
        'tf2_geometry_msgs': module(
            'tf2_geometry_msgs', do_transform_pose_stamped=_do_transform_pose_stamped),
        'cv_bridge': module('cv_bridge', CvBridge=_Bridge),
    }


@contextmanager
def _swapped_modules(replacements):
    saved = {name: sys.modules.get(name) for name in replacements}
    sys.modules.update(replacements)
    try:
        yield
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def load_detector(state):
    """Import grounding_dino_node.py bound to the stand-ins in ``state``."""
    package_root = str(ROOT / 'homeplus_vision')
    if package_root not in sys.path:
        sys.path.insert(0, package_root)
    spec = importlib.util.spec_from_file_location('detector_replay_node', NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    with _swapped_modules(_stub_modules(state)):
        spec.loader.exec_module(module)
    return module


# ----------------------------------------------------------------------------
# Inputs

def load_params(path=DEFAULT_PARAMS):
    payload = yaml.safe_load(Path(path).read_text()) or {}
    for value in payload.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            return dict(value['ros__parameters'])
    return {}


def load_npz(path):
    data = np.load(path, allow_pickle=False)
    frame = {key: data[key] for key in data.files}
    for key in ('depth_encoding', 'caption', 'camera_frame', 'world_frame'):
        if key in frame:
            frame[key] = str(frame[key])
    for key in ('camera_width', 'camera_height', 'stamp_ns', 'depth_stamp_ns', 'task_id'):
        if key in frame:
            frame[key] = int(frame[key])
    return frame


def save_npz(frame, path):
    np.savez_compressed(path, **{k: v for k, v in frame.items() if v is not None})


def load_bag(path, params, index=0):
    """Pick the ``index``-th color frame of a ros2 bag with its nearest depth and TF."""
    import rosbag2_py
    import tf2_ros
    from collections import deque
    from cv_bridge import CvBridge
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message

    path = Path(path)
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())
    storage = metadata['rosbag2_bagfile_information'].get('storage_identifier', 'sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    color_topic, depth_topic = params['image_topic'], params['depth_topic']
    info_topic = params['camera_info_topic']
    for topic in (color_topic, depth_topic, info_topic):
        if topic not in types_by_topic:
            raise SystemExit(f'{topic} not in bag; topics: {sorted(types_by_topic)}')

    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    bridge = CvBridge()
    tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3600))
    depths = deque(maxlen=120)
    color = info = None
    seen = 0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in (color_topic, depth_topic, info_topic, '/tf', '/tf_static'):
            continue
        msg = deserialize_message(raw, get_message(types_by_topic[topic]))
        if topic in ('/tf', '/tf_static'):
            for transform in msg.transforms:
                if topic == '/tf_static':
                    tf_buffer.set_transform_static(transform, 'bag')
                else:
                    tf_buffer.set_transform(transform, 'bag')
            if color is not None and stamp_ns(msg.transforms[0].header) > stamp_ns(color.header) + 1e9:
                break
        elif topic == info_topic:
            info = info or msg
        elif topic == depth_topic:
            depths.append(msg)
        elif color is None:
            if seen == index:
                color = msg
            seen += 1
    if color is None or info is None or not depths:
        raise SystemExit(f'bag has {seen} color frames; need index {index}, camera_info and depth')

    color_ns = stamp_ns(color.header)
    depth = min(depths, key=lambda d: abs(stamp_ns(d.header) - color_ns))
    camera_frame = params.get('camera_frame') or info.header.frame_id
    world_frame = params.get('world_frame', 'world')
    try:
        tf = tf_buffer.lookup_transform(world_frame, camera_frame, Time.from_msg(color.header.stamp))
        world_from_camera = np.eye(4)
        world_from_camera[:3, :3] = _quat_matrix(tf.transform.rotation)
        t = tf.transform.translation
        world_from_camera[:3, 3] = [t.x, t.y, t.z]
        tf_error = None
    except Exception as exc:
        world_from_camera, tf_error = None, str(exc)
    return {
        'color': bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
        'depth': bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'),
        'depth_encoding': depth.encoding,
        'depth_stamp_ns': stamp_ns(depth.header),
        'K': np.asarray(info.k, dtype=np.float64),
        'camera_width': int(info.width),
        'camera_height': int(info.height),
        'T_world_camera': world_from_camera,
        'tf_error': tf_error,
        'stamp_ns': color_ns,
        'camera_frame': camera_frame,
        'world_frame': world_frame,
    }


def _transform_msg(world_from_camera, frame):
    rotation = np.asarray(world_from_camera)[:3, :3]
    translation = np.asarray(world_from_camera)[:3, 3]
    # Matrix -> quaternion (Shepperd); the detector converts it back.
    trace = np.trace(rotation)
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s,
             (rotation[1, 0] - rotation[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        q = [0.0, 0.0, 0.0, (rotation[k, j] - rotation[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
    msg = _TransformStamped()
    msg.header.frame_id = frame.get('world_frame', 'world')
    msg.child_frame_id = frame.get('camera_frame', '')
    msg.transform.translation = SimpleNamespace(x=translation[0], y=translation[1], z=translation[2])
    msg.transform.rotation = SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
    return msg


# ----------------------------------------------------------------------------
# Replay

def run_replay(frame, server_url='', params=None, overrides=None, point=None,
               mask=None, motion_status=True, frame_age_sec=0.2, echo_logs=True):
    """Run one detector cycle on ``frame``; return diagnostics and outputs.

    point: (u, v) in the 640x480 inference image, replaces MolmoPoint.
    mask:  HxW array (nonzero = object) in the 640x480 image, replaces SAM2.
    motion_status: True sends /arduino/status=true before the frame, False
        sends nothing (reproduces the node's start-paused state).
    """
    stamp_ns = int(frame.get('stamp_ns') or 1_000_000_000_000)
    state = {'logs': [], 'published': {}, 'echo_logs': echo_logs,
             'now_ns': stamp_ns, 'tf': None, 'tf_error': frame.get('tf_error')}
    module = load_detector(state)
    node_class = module.MolmoPointNode

    scratch = Path(tempfile.mkdtemp(prefix='detector_replay_'))
    values = dict(load_params() if params is None else params)
    values.update(overrides or {})
    if frame.get('caption') and 'caption' not in (overrides or {}):
        values['caption'] = frame['caption']
    if frame.get('task_id') and 'task_id' not in (overrides or {}):
        values['task_id'] = frame['task_id']
    values.update(target_audit_dir=str(scratch), model_cache_dir=str(scratch / 'models'),
                  debug_dump_dir='')
    if server_url:
        values['remote_inference_url'] = server_url
    elif point is not None:
        # No server: take the remote code path, but answer it locally.
        values['remote_inference_url'] = 'replay://manual'
        node_class._check_remote_health = lambda self: None
    else:
        raise ValueError('need a server URL or a manual --point')
    state['overrides'] = values

    node = node_class()
    if point is not None:
        node._remote_point = lambda image, caption: (
            caption, 'manual point', [[0, 0, float(point[0]), float(point[1])]])
    if mask is not None:
        manual = np.asarray(mask) > 0

        def segment(image, _point):
            if manual.shape != image.shape[:2]:
                raise RuntimeError(f'manual mask {manual.shape} != frame {image.shape[:2]}')
            return manual[None], np.asarray([1.0], dtype=np.float32)
        node._remote_segmentation = segment
    elif not server_url:
        raise ValueError('a manual --point without a server also needs --mask')

    world_from_camera = frame.get('T_world_camera')
    if world_from_camera is not None:
        state['tf'] = _transform_msg(world_from_camera, frame)

    settle_ns = int(float(values.get('motion_settle_sec', 0.7)) * 1e9)
    if motion_status and getattr(node, 'pause_during_motion', False):
        state['now_ns'] = stamp_ns - settle_ns - 1_000_000_000
        node.motion_status_callback(_Data(True))
    state['now_ns'] = stamp_ns + int(frame_age_sec * 1e9)

    info = _CameraInfo()
    info.header.frame_id = frame.get('camera_frame', '')
    info.width, info.height = int(frame['camera_width']), int(frame['camera_height'])
    info.k = [float(v) for v in np.asarray(frame['K']).ravel()]
    node.camera_info_callback(info)

    bridge = _Bridge()
    depth = np.asarray(frame['depth'])
    depth_msg = bridge.cv2_to_imgmsg(
        depth, frame.get('depth_encoding') or ('16UC1' if depth.dtype == np.uint16 else '32FC1'))
    depth_ns = int(frame.get('depth_stamp_ns') or stamp_ns)
    depth_msg.header = _Header(_Time(nanoseconds=depth_ns).to_msg(), info.header.frame_id)
    node.depth_callback(depth_msg)

    color_msg = bridge.cv2_to_imgmsg(np.asarray(frame['color']), 'bgr8')
    color_msg.header = _Header(_Time(nanoseconds=stamp_ns).to_msg(), info.header.frame_id)
    node.image_callback(color_msg)

    node.detect_loop()

    published = state['published']
    diagnostics = [json.loads(m.data) for m in published.get(values.get(
        'diagnostics_topic', '/molmopoint/diagnostics'), [])]
    debug = published.get(values.get('debug_image_topic', '/molmopoint/debug_image'), [])
    poses = published.get(values.get('pose_topic', '/molmopoint_pose_world'), [])
    points = published.get(values.get('mask_center_topic', '/molmopoint/point'), [])
    return {
        'diagnostics': diagnostics[-1] if diagnostics else None,
        'debug_image': debug[-1].array if debug else None,
        'pose_world': ([poses[-1].pose.position.x, poses[-1].pose.position.y,
                        poses[-1].pose.position.z] if poses else None),
        'mask_center': json.loads(points[-1].data) if points else None,
        'logs': state['logs'],
        'published_topics': {topic: len(msgs) for topic, msgs in published.items()},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', help='.npz frame or ros2 bag directory')
    parser.add_argument('--server', default='', help='remote inference URL, e.g. http://192.168.64.1:8765')
    parser.add_argument('--params', default=str(DEFAULT_PARAMS), help='detector params YAML')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                        help='override a detector parameter (YAML value)')
    parser.add_argument('--caption', help='pointing caption (default: frame/task preset)')
    parser.add_argument('--task-id', type=int)
    parser.add_argument('--point', type=float, nargs=2, metavar=('U', 'V'),
                        help='skip MolmoPoint; use this 640x480 pixel')
    parser.add_argument('--mask', help='skip SAM2; mono PNG mask at 640x480')
    parser.add_argument('--tf', type=float, nargs=7, metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'),
                        help='world<-camera transform (overrides the frame/bag TF)')
    parser.add_argument('--no-tf', action='store_true', help='simulate a TF lookup failure')
    parser.add_argument('--no-motion-status', action='store_true',
                        help='never send /arduino/status=true (node starts paused)')
    parser.add_argument('--frame-age', type=float, default=0.2,
                        help='ROS clock minus frame stamp at detection time (s)')
    parser.add_argument('--bag-index', type=int, default=0, help='color frame index in a bag')
    parser.add_argument('--to-npz', help='also save the loaded frame as .npz')
    parser.add_argument('--debug-out', help='write the debug image PNG here')
    parser.add_argument('--quiet', action='store_true', help='do not echo node logs')
    args = parser.parse_args(argv)

    params = load_params(args.params)
    overrides = {}
    for item in args.set:
        key, _, value = item.partition('=')
        overrides[key.strip()] = yaml.safe_load(value)
    if args.caption:
        overrides['caption'] = args.caption
    if args.task_id is not None:
        overrides['task_id'] = args.task_id

    source = Path(args.input)
    if source.is_dir():
        frame = load_bag(source, {**params, **overrides}, args.bag_index)
    else:
        frame = load_npz(source)
    if args.to_npz:
        save_npz({k: v for k, v in frame.items() if k != 'tf_error'}, args.to_npz)
        print(f'saved frame to {args.to_npz}')
    if args.tf:
        x, y, z, qx, qy, qz, qw = args.tf
        world_from_camera = np.eye(4)
        world_from_camera[:3, :3] = _quat_matrix(SimpleNamespace(x=qx, y=qy, z=qz, w=qw))
        world_from_camera[:3, 3] = [x, y, z]
        frame['T_world_camera'] = world_from_camera
    if args.no_tf:
        frame['T_world_camera'] = None
    elif frame.get('T_world_camera') is None and not frame.get('tf_error'):
        print('WARNING: frame has no T_world_camera; using identity (world == camera '
              'optical frame), so task-1 approach geometry is not meaningful. Pass --tf.')
        frame['T_world_camera'] = np.eye(4)

    mask = None
    if args.mask:
        mask = cv2.imread(args.mask, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise SystemExit(f'cannot read mask {args.mask}')
    if not args.server and args.point is None:
        parser.error('give --server, or --point (and --mask) to bypass the models')

    color = np.asarray(frame['color'])
    depth = np.asarray(frame['depth'])
    print(f'input: color {color.shape} depth {depth.shape} {depth.dtype} '
          f"K={np.round(np.asarray(frame['K']).ravel()[[0, 4, 2, 5]], 1).tolist()} "
          f"T_world_camera={'yes' if frame.get('T_world_camera') is not None else 'NO'}")
    print('node log:')
    result = run_replay(
        frame, server_url=args.server, params=params, overrides=overrides,
        point=args.point, mask=mask, motion_status=not args.no_motion_status,
        frame_age_sec=args.frame_age, echo_logs=not args.quiet)

    diag = result['diagnostics'] or {}
    print('\ndiagnostics:')
    print(json.dumps(diag, indent=2, sort_keys=True))
    print(f"\nRESULT: stage={diag.get('stage')} reason={diag.get('reason')}")
    if result['pose_world'] is not None:
        print('pose_world: ' + ', '.join(f'{v:.4f}' for v in result['pose_world']))
    if args.debug_out and result['debug_image'] is not None:
        cv2.imwrite(args.debug_out, result['debug_image'])
        print(f'debug image: {args.debug_out}')
    return 0 if result['pose_world'] is not None else 1


if __name__ == '__main__':
    sys.exit(main())
