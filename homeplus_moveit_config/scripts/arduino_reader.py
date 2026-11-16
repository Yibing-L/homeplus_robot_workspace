#!/usr/bin/env python3
"""Buffered trajectory bridge (STATE4 firmware for streaming). Only hardware STATE2 messages publish robot state.

Base and shoulder/elbow are measured; gripper/hand/wrist are PWM estimates.
Requires the matching ARDUINO_FIRMWARE_UPDATED.txt; legacy firmware cannot move.
"""
from collections import deque
import binascii
import argparse
import glob
import json
import math
import time
import uuid

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

JOINT_NAMES = ["base_x", "base_y", "base_theta", "joint_grip_L", "joint_hand",
               "joint_wrist", "joint_elbow", "joint_shoulder"]
WHEEL_JOINT_NAMES = ["joint_wheel_FL", "joint_wheel_FR", "joint_wheel_RL", "joint_wheel_RR"]
MAX_COMMANDS = 45
MAX_PAYLOAD_BYTES = 2042  # Reserve five bytes for the serial CRC field.
# SI bounds match the model and firmware, including the elbow margin.
LIMITS = [(-10, 10), (-10, 10), (-math.inf, math.inf), (0, math.pi / 2),
          (math.radians(-70), math.radians(80)), (-math.pi / 2, math.pi / 2),
          (math.radians(-35), math.radians(140)), (math.radians(-30), math.radians(150))]


# Live jog (console teleop): one open-ended STR stream per jog session, one point per packet.
# The firmware interpolates toward each point and stops on BUFFER_UNDERRUN if points stop
# arriving, so the lookahead is also the firmware-side dead-man time. No firmware change.
JOG_LOOKAHEAD_MS = 300        # each point is due this far after "now" on the firmware clock
JOG_INPUT_TIMEOUT_S = 0.15    # no setpoint for this long: finish the stream (hold the last point)
JOG_MAX_POINT_MS = 110000     # firmware rejects point times > 120 s; end the session before that
JOG_UNDERRUN_PAD_MS = 20      # extra lookahead added whenever the firmware buffer nearly drained
# Defence in depth against a console bug: velocity between consecutive streamed points,
# hardware units per second (cm, cm, deg, deg...), about twice the console's own limits.
JOG_MAX_RATE = [25.0, 25.0, 40.0, 180.0, 80.0, 120.0, 30.0, 25.0]
# A session must start from where the robot is: first point vs measured (hardware units).
JOG_START_TOLERANCE = [2.0, 2.0, 1.5, 10.0, 10.0, 10.0, 3.0, 3.0]


def from_hardware(values):
    if len(values) != 8 or not all(math.isfinite(v) for v in values):
        raise ValueError("invalid hardware state")
    return [values[0] / 100, values[1] / 100] + [math.radians(v) for v in values[2:]]


def to_hardware(si):
    """SI joint values in JOINT_NAMES order (m, rad) -> firmware units (cm, deg)."""
    return [si[0] * 100, si[1] * 100] + [math.degrees(v) for v in si[2:8]]


def jog_point_text(hw, t_ms):
    return ' '.join(f'{v:.2f}' for v in hw) + f' 0 0 {int(t_ms)}'


def validate_jog_setpoint(si):
    """8 finite SI values inside the bridge's joint limits, or ValueError."""
    q = [float(v) for v in si]
    if len(q) != 8 or not all(math.isfinite(v) for v in q):
        raise ValueError('need 8 finite joint values')
    for v, (lo, hi), name in zip(q, LIMITS, JOINT_NAMES):
        if not lo - 1e-5 <= v <= hi + 1e-5:
            raise ValueError(f'{name} outside joint limits')
    return q


def jog_rate_violation(prev_hw, hw, dt_s):
    """Name of the first joint whose streamed velocity exceeds JOG_MAX_RATE, else None."""
    dt_s = max(dt_s, 1e-3)
    for name, a, b, rate in zip(JOINT_NAMES, prev_hw, hw, JOG_MAX_RATE):
        if abs(b - a) / dt_s > rate:
            return f'{name} {abs(b - a) / dt_s:.1f}/s > {rate}/s'
    return None


def validate_command(command, streaming=False):
    """Validate the entire queue before any bytes are sent to hardware."""
    prefix, command_id, queue = command.split(" ", 2)
    if prefix != "CMD" or not 0 < int(command_id) < 2**32:
        raise ValueError("expected CMD <uint32 id> <timed queue>")
    if len(command.encode()) > (1000000 if streaming else MAX_PAYLOAD_BYTES) or '\n' in command or '\r' in command:
        raise ValueError("queue exceeds firmware RX capacity or contains a newline")
    points = queue.split(',')
    if not 1 <= len(points) <= (10000 if streaming else MAX_COMMANDS):
        raise ValueError("queue exceeds firmware waypoint capacity")
    previous = -1
    for i, point in enumerate(points):
        values = [float(v) for v in point.split()]
        if len(values) != 11 or not all(math.isfinite(v) for v in values):
            raise ValueError("each waypoint needs 10 positions plus milliseconds")
        t = values[10]
        if t != int(t) or t <= previous or t > 120000 or (i == 0 and t != 0):
            raise ValueError("waypoint times must increase from zero")
        previous = t
        if values[8:10] != [0.0, 0.0]:
            raise ValueError("unmodelled linear actuators must remain zero")
        for q, (lo, hi) in zip(from_hardware(values[:8]), LIMITS):
            if not lo - 1e-5 <= q <= hi + 1e-5:
                raise ValueError("waypoint exceeds joint limits")
    return int(command_id), previous / 1000.0


class ArduinoBridge(Node):
    def __init__(self, port="auto", baud=115200, read_hz=100.0):
        super().__init__("arduino_bridge")
        self.port, self.baud = port, baud
        self.ser = None
        self.rx = bytearray()
        self.active_id = None
        self.stream = None
        self.tx = None
        self.streaming_capable = False
        self.deadline = 0.0
        self.last_feedback = 0.0
        self.state_history = deque(maxlen=30)
        self.last_connect = -10.0
        self.last_ping = 0.0
        self.compatible = False
        self.ready = False
        self.serial_bytes_received = 0
        self.last_serial_line = ''
        # Identity for the operator console: session_id changes on every port open, boot_id on every
        # firmware boot. Same boot across a reopen = odometry intact; new boot = odometry re-zeroed.
        self.session_id = None
        self.opened_wall = None
        self.boot_id = None
        self.zero_supported = False
        self.zero_pending = False
        self.zero_count = 0
        self.firmware_restarts = 0
        self.last_error = None
        self.id_requested = False
        self.jog = None
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.hardware_state_pub = self.create_publisher(JointState, '/hardware/joint_states', 10)
        self.ack_pub = self.create_publisher(String, '/arduino/ack', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(Bool, '/arduino/status', latched)
        self.ready_pub = self.create_publisher(Bool, '/arduino/ready', latched)
        self.info_pub = self.create_publisher(String, '/arduino/bridge_info', latched)
        self.jog_status_pub = self.create_publisher(String, '/arduino/jog_status', 10)
        self._set_ready(False)
        self.status_pub.publish(Bool(data=False))
        self.create_subscription(String, '/arduino/command_queue', self._command_queue_callback, 10)
        self.create_subscription(Bool, '/arduino/stop', self._stop_callback, 10)
        self.create_subscription(Bool, '/arduino/zero', self._zero_callback, 10)
        self.create_subscription(String, '/arduino/jog', self._jog_callback, 10)
        self.create_timer(1.0 / max(20, read_hz), self._read_loop)
        self.create_timer(5.0, self._report_connection)

    def _report_connection(self):
        # Repeat the current readiness for late subscribers as well as logging
        # startup failures that previously stayed silent indefinitely.
        self.ready_pub.publish(Bool(data=self.ready))
        if self.ser is None:
            self.get_logger().warning('Arduino serial port is not connected; retrying.')
        elif not self.compatible:
            self.get_logger().warning(
                f'No fresh firmware state feedback at {self.baud} baud: '
                f'{self.serial_bytes_received} bytes received since opening; '
                f'last line={self.last_serial_line!r}; '
                f'buffered bytes={len(self.rx)}, prefix={bytes(self.rx[:24])!r}. '
                'Waiting without transmitting until firmware telemetry arrives.')

    def _publish_info(self):
        self.info_pub.publish(String(data=json.dumps({
            'session_id': self.session_id, 'boot_id': self.boot_id, 'port': self.port,
            'opened_wall': self.opened_wall, 'compatible': self.compatible,
            'protocol': 'STATE4' if self.streaming_capable else None, 'last_error': self.last_error,
            'firmware_restarts': self.firmware_restarts, 'zero_supported': self.zero_supported,
            'zero_count': self.zero_count, 'jog': True, 'jog_lookahead_ms': JOG_LOOKAHEAD_MS})))

    def _zero_callback(self, msg):
        """ZERO: odometry and held base target := current pose, without resetting the board."""
        if not msg.data:
            return
        if not self.zero_supported:
            self.ack_pub.publish(String(data='ERROR 0 ZERO_UNSUPPORTED firmware has no ZERO command'))
            return
        if self.ser is None or not self.compatible or not self.ready or self.active_id is not None \
                or getattr(self, 'tx', None) is not None or self.zero_pending:
            self.ack_pub.publish(String(data='ERROR 0 ZERO_REJECTED bridge not idle and ready'))
            return
        self.zero_pending = True
        self.ser.write(b'\nZERO\n')
        self.get_logger().info('ZERO sent: odometry and held base target to the current pose')

    def _set_ready(self, ready):
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))

    def _fail(self, reason):
        self.last_error = reason
        if self.jog is not None:
            self._jog_status(self.jog, 'failed', reason)
            self.jog = None
        if self.active_id is not None:
            self.ack_pub.publish(String(data=f'ERROR {self.active_id} {reason}'))
        self.active_id = None
        self.stream = None
        self.tx = None
        self._set_ready(False)
        # An error is not a settled pose. Perception remains paused until a
        # subsequent successful command/explicit reinitialization.
        self.status_pub.publish(Bool(data=False))
        self.state_history.clear()

    def _stop_callback(self, msg):
        if msg.data:
            if self.ser:
                self.ser.write(b'\nSTOP\n')
            if self.active_id is not None:
                self._fail('HOST_STOP')

    def _read_loop(self):
        now = time.monotonic()
        try:
            if self.ser is None:
                if now - self.last_connect < 2.0:
                    return
                self.last_connect = now
                paths = sorted(glob.glob('/dev/serial/by-id/*')) or sorted(glob.glob('/dev/ttyACM*'))
                port = self.port if self.port != 'auto' else (paths[0] if paths else None)
                if port is None:
                    return
                self.ser = serial.Serial(port, self.baud, timeout=0, write_timeout=1)
                self.compatible = False
                self.serial_bytes_received = 0
                self.last_serial_line = ''
                self.session_id = uuid.uuid4().hex[:12]
                self.opened_wall = time.time()
                self.id_requested = False
                self.zero_pending = False
                self._publish_info()
                self.get_logger().info(f'Connected {port} at {self.baud} baud; waiting for firmware state feedback')
            # Opening the serial port can reset the Mega. Do not send ASCII
            # into its bootloader: firmware streams STATE2 without a request.
            # Begin heartbeats only after that proves the sketch is running.
            if self.compatible and getattr(self, 'tx', None) is None and now - self.last_ping > 0.25:
                self.ser.write(b'PING\n')
                self.last_ping = now
            received = self.ser.read(self.ser.in_waiting)
            self.serial_bytes_received += len(received)
            self.rx.extend(received)
            if len(self.rx) > 8192:
                raise ValueError('unterminated serial telemetry')
            while b'\n' in self.rx:
                line, _, rest = self.rx.partition(b'\n')
                self.rx = bytearray(rest)
                self.last_serial_line = bytes(line[:160]).decode('ascii', errors='replace')
                self._handle_line(line.decode('ascii', errors='strict').strip())
            self._pump_tx()
            jog = self.jog
            if jog is not None and not jog['ending'] and now - jog['last_rx'] > JOG_INPUT_TIMEOUT_S:
                self._jog_end(jog, 'input timeout')
            if self.compatible and now - self.last_feedback > 1.0:
                self.ser.write(b'\nSTOP\n')
                self._fail('STALE_FEEDBACK')
                self.compatible = False
            if self.active_id is not None and now > self.deadline:
                self.ser.write(b'\nSTOP\n')
                self._fail('HOST_TIMEOUT')
        except (serial.SerialException, OSError, ValueError) as exc:
            self.get_logger().error(f'Arduino serial failure: {exc}; last line={self.last_serial_line!r}')
            self._fail(f'SERIAL {exc}')
            if self.ser:
                self.ser.close()
            self.ser = None
            self.rx.clear()
            self.compatible = False

    def _handle_line(self, line):
        fields = line.split()
        if not fields:
            return
        if fields[0] in ('STATE2', 'STATE3', 'STATE4'):
            self.streaming_capable = fields[0] == 'STATE4'
            if len(fields) != 10:
                raise ValueError('bad STATE2 field count')
            positions = from_hardware([float(v) for v in fields[2:]])
            first = not self.compatible
            self.compatible = True
            self.last_feedback = time.monotonic()
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = JOINT_NAMES + WHEEL_JOINT_NAMES
            js.position = positions + [0.0] * 4
            self.joint_state_pub.publish(js)
            self.hardware_state_pub.publish(js)
            self.state_history.append((self.last_feedback, positions))
            settled = len(self.state_history) >= 8 and self.state_history[-1][0] - self.state_history[0][0] >= 0.7
            if settled:
                for i in range(8):
                    tolerance = 0.001 if i < 2 else math.radians(0.15)
                    if max(q[i] for _,q in self.state_history) - min(q[i] for _,q in self.state_history) > tolerance:
                        settled = False
                        break
            if self.active_id is None and not self.ready and settled:
                self._set_ready(True)
                self.status_pub.publish(Bool(data=True))
            if first:
                # State transport is healthy. Idle boot may acquire perception;
                # execution additionally checks measured stillness on the host.
                self.get_logger().info(f'{fields[0]} detected; waiting for measured stillness')
                self._publish_info()
            if not self.id_requested and getattr(self, 'tx', None) is None:
                # Ask which boot this is (a reopen without DTR keeps the same boot). Older
                # firmware ignores it, and boot_id stays unknown.
                self.ser.write(b'ID\n')
                self.id_requested = True
            return
        if fields[0] == 'BOOT':
            if len(fields) >= 2:
                boot = fields[1]
                if self.boot_id is not None and boot != self.boot_id:
                    self.get_logger().warning(f'Firmware boot changed {self.boot_id} -> {boot}: odometry was reset')
                self.boot_id = boot
                self.zero_supported = 'ZERO' in fields[2:]
                self._publish_info()
            return
        if fields[0] == 'ZEROED':
            self.zero_pending = False
            self.zero_count += 1
            self.state_history.clear()  # positions jumped; stillness must be re-established
            self.ack_pub.publish(String(data=line))
            self.get_logger().info(f'ZEROED: odometry and held base target now at the current pose ({line})')
            self._publish_info()
            return
        if fields[0] == 'ERROR' and len(fields) >= 3 and fields[2] == 'ZERO_BUSY':
            self.zero_pending = False
            self.ack_pub.publish(String(data=line))
            return
        if fields[0] in ('READY2', 'READY3', 'READY4'):
            if self.active_id is not None:
                self._fail('FIRMWARE_RESTART')
            self.compatible = False
            self.firmware_restarts += 1
            self.boot_id = None  # the BOOT line that follows names the new boot
            self.zero_pending = False
            self._publish_info()
            return
        if fields[0] == 'RX4':
            tx = getattr(self, 'tx', None)
            if tx is not None and len(fields) == 2:
                consumed = int(fields[1])
                if consumed == tx['sent']:
                    tx['acked'] = consumed
            return
        if fields[0] == 'BUF3':
            if len(fields) != 4:
                raise ValueError('bad stream credit')
            cid, sequence, free = map(int, fields[1:])
            if cid == self.active_id and getattr(self, 'stream', None) is not None:
                if self.stream.get('jog'):
                    self._jog_refill(sequence, free)
                else:
                    self._refill_stream(sequence, free)
            return
        if fields[0] not in ('ACK', 'DONE', 'ERROR') or len(fields) < 2:
            return
        try:
            command_id = int(fields[1])
        except ValueError:
            return  # Legacy messages never acknowledge v2 motion.
        if command_id != self.active_id:
            self.get_logger().warning(f'Ignoring stale/unmatched acknowledgement: {line}')
            return
        if fields[0] == 'DONE':
            if time.monotonic() - self.last_feedback > 0.2:
                self._fail('DONE_WITHOUT_FEEDBACK')
                return
            if getattr(self, 'stream', None) is not None and self.stream['offset'] < len(self.stream['points']):
                self.ser.write(b'\nSTOP\n')
                self._fail('PREMATURE_DONE')
                return
            self.stream = None
            if self.jog is not None:
                self._jog_status(self.jog, 'done', 'settled')
                self.jog = None
            self.ack_pub.publish(String(data=line))
            self.active_id = None
            self._set_ready(True)
            self.status_pub.publish(Bool(data=True))
        elif fields[0] == 'ERROR':
            self._fail(' '.join(fields[2:]))
        elif self.jog is None:  # per-packet jog ACKs (~15 Hz) would only flood /arduino/ack
            self.ack_pub.publish(String(data=line))

    def _command_queue_callback(self, msg):
        command = msg.data.strip()
        transit = command.startswith('TRANSIT ')
        streaming = transit or command.startswith('STREAM ')
        if streaming:
            command = 'CMD ' + command.split(' ', 1)[1]
        try:
            command_id, duration = validate_command(command, streaming=streaming)
            if streaming and not getattr(self, 'streaming_capable', False):
                raise ValueError('upload matching STATE4 flow-controlled streaming firmware first')
        except (ValueError, OverflowError) as exc:
            fields = command.split()
            cid = fields[1] if len(fields) > 1 and fields[1].isdigit() else '0'
            self.ack_pub.publish(String(data=f'ERROR {cid} INVALID_COMMAND {exc}'))
            return
        if self.active_id is not None or not self.ready or not self.compatible or time.monotonic() - self.last_feedback > 0.5:
            self.ack_pub.publish(String(data=f'ERROR {command_id} NOT_READY_OR_BUSY'))
            return
        self.active_id = command_id
        self.deadline = time.monotonic() + duration + 15.0
        self.status_pub.publish(Bool(data=False))
        try:
            if streaming:
                self.stream = dict(points=command.split(' ', 2)[2].split(','), offset=0, sequence=0, transit=transit)
                self._refill_stream(0, MAX_COMMANDS)
                return
            # Pace AVR RX bursts. No acknowledgement is consumed during writes.
            prefix, cid, queue = command.split(' ', 2)
            crc = binascii.crc_hqx(queue.encode('ascii'), 0xffff)
            payload = f'{prefix} {cid} {crc:04x} {queue}\n'.encode('ascii')
            for offset in range(0, len(payload), 16):
                if self.ser.write(payload[offset:offset + 16]) != len(payload[offset:offset + 16]):
                    raise serial.SerialException('short write')
                self.ser.flush()
                time.sleep(0.003)
            # Drain queued telemetry on the next timer before checking freshness.
        except (serial.SerialException, OSError, ValueError) as exc:
            try:
                self.ser.write(b'\nSTOP\n')
            except (serial.SerialException, OSError):
                pass  # Firmware heartbeat watchdog stops a lost connection.
            self._fail(f'WRITE {exc}')

    # ------------------------------------------------------------------ live jog (console teleop)
    def _jog_status(self, jog, state, reason=''):
        self.jog_status_pub.publish(String(data=json.dumps({
            'session': jog['session'], 'cid': jog.get('cid'), 'state': state, 'reason': reason,
            'lookahead_ms': JOG_LOOKAHEAD_MS + jog.get('pad_ms', 0), 'points': jog.get('points', 0)})))

    def _jog_callback(self, msg):
        """/arduino/jog JSON: {"op": "point", "session": s, "q": [8 SI values, JOINT_NAMES order]}
        or {"op": "end", "session": s}. The first point of a new session opens a stream at the
        measured pose; later points replace the pending setpoint (only the newest is streamed)."""
        try:
            m = json.loads(msg.data)
            op, session = m['op'], str(m['session'])
        except (ValueError, KeyError, TypeError):
            return
        jog = self.jog
        if op == 'end':
            if jog is not None and jog['session'] == session and not jog['ending']:
                self._jog_end(jog, 'console end')
            return
        if op != 'point':
            return
        try:
            q = validate_jog_setpoint(m['q'])
        except (ValueError, KeyError, TypeError) as exc:
            if jog is not None and jog['session'] == session:
                self._jog_abort(f'JOG_BAD_SETPOINT {exc}')
            else:
                self._jog_status({'session': session}, 'rejected', str(exc))
            return
        if jog is None:
            self._jog_start(session, q)
            return
        if jog['session'] != session:
            self._jog_status({'session': session}, 'rejected', 'another jog session is active')
            return
        if jog['ending']:
            return
        jog['q'] = q
        jog['fresh'] = True
        jog['last_rx'] = time.monotonic()

    def _jog_start(self, session, q):
        now = time.monotonic()
        reason = None
        if self.ser is None or not self.compatible:
            reason = 'no firmware link'
        elif not getattr(self, 'streaming_capable', False):
            reason = 'firmware is not STATE4 streaming firmware'
        elif self.active_id is not None or getattr(self, 'tx', None) is not None:
            reason = 'bridge busy with another command'
        elif not self.ready:
            reason = 'bridge not ready (robot not still yet, or after an error)'
        elif now - self.last_feedback > 0.2 or not self.state_history:
            reason = 'joint feedback stale'
        if reason is None:
            measured = to_hardware(self.state_history[-1][1])
            hw = to_hardware(q)
            for name, a, b, tol in zip(JOINT_NAMES, measured, hw, JOG_START_TOLERANCE):
                if abs(a - b) > tol:
                    reason = f'first point {name} is {abs(a - b):.1f} from the measured pose (max {tol})'
                    break
        if reason is not None:
            self._jog_status({'session': session}, 'rejected', reason)
            return
        cid = (time.time_ns() // 1000) % (2**32 - 1) + 1
        self.active_id = cid
        self.deadline = now + JOG_MAX_POINT_MS / 1000.0 + 15.0
        self.status_pub.publish(Bool(data=False))
        self.stream = dict(points=[], offset=0, sequence=0, transit=False, jog=True)
        self.jog = dict(session=session, cid=cid, q=q, sent_hw=hw, last_t=JOG_LOOKAHEAD_MS, t0=now,
                        last_rx=now, fresh=False, ending=False, final_sent=False, pad_ms=0, points=2)
        # Two points: hold the current pose now, and again one lookahead later.
        try:
            self._jog_send([jog_point_text(hw, 0), jog_point_text(hw, JOG_LOOKAHEAD_MS)], final=False)
        except (serial.SerialException, OSError, ValueError) as exc:
            self._jog_abort(f'WRITE {exc}')
            return
        self._jog_status(self.jog, 'active')
        self.get_logger().info(f'Jog session {session} started as stream {cid}')

    def _jog_send(self, points, final):
        stream = self.stream
        body = f"{stream['sequence']} {1 if final else 0} " + ','.join(points)
        crc = binascii.crc_hqx(body.encode('ascii'), 0xffff)
        payload = f'STR {self.active_id} {crc:04x} {body}\n'.encode('ascii')
        if len(payload) >= 2048:
            raise ValueError('jog packet exceeds firmware RX capacity')
        if getattr(self, 'tx', None) is not None:
            raise ValueError('overlapping serial packets')
        stream['sequence'] += 1
        self.tx = dict(payload=payload, sent=0, acked=0, last_progress=time.monotonic())
        self._pump_tx()

    def _jog_end(self, jog, why):
        """Finish the stream: the next credit sends a final point equal to the last streamed point,
        so the robot stops where it was already heading (at most one lookahead away) and settles."""
        jog['ending'] = True
        self._jog_status(jog, 'ending', why)

    def _jog_abort(self, reason):
        if self.ser is not None:
            self.ser.write(b'\nSTOP\n')
        self._fail(reason)

    def _jog_refill(self, sequence, free):
        jog, stream = self.jog, self.stream
        if jog is None or sequence != stream['sequence'] or getattr(self, 'tx', None) is not None or jog['final_sent']:
            return
        now = time.monotonic()
        # BUF3 free counts include the interpolation anchor: 44 free = the firmware is on its last point.
        if MAX_COMMANDS - free <= 1 and not jog['ending']:
            jog['pad_ms'] = min(jog['pad_ms'] + JOG_UNDERRUN_PAD_MS, 200)
        due = int((now - jog['t0']) * 1000) + JOG_LOOKAHEAD_MS + jog['pad_ms']
        if not jog['ending'] and due > JOG_MAX_POINT_MS:
            self._jog_end(jog, 'session length limit (the next input starts a new session)')
        if jog['ending']:
            self._jog_send([jog_point_text(jog['sent_hw'], jog['last_t'] + 20)], final=True)
            jog['final_sent'] = True
            jog['last_t'] += 20
            return
        if not jog['fresh']:
            return  # nothing new: the buffer drains; the input watchdog ends the session
        hw = to_hardware(jog['q'])
        t = max(jog['last_t'] + 10, due)
        bad = jog_rate_violation(jog['sent_hw'], hw, (t - jog['last_t']) / 1000.0)
        if bad:
            self._jog_abort(f'JOG_RATE_GUARD {bad}')
            return
        self._jog_send([jog_point_text(hw, t)], final=False)
        jog['sent_hw'], jog['last_t'], jog['fresh'] = hw, t, False
        jog['points'] += 1

    def _refill_stream(self, sequence, free):
        """One outstanding packet; credits include the retained interpolation point."""
        stream = self.stream
        if sequence != stream['sequence']:
            return  # Old credit cannot authorize another write.
        if not 0 <= free <= MAX_COMMANDS:
            raise ValueError('invalid stream capacity')
        begin = stream['offset']
        count = min(24 if begin == 0 else 12, len(stream['points']) - begin)
        if count == 0 or free < count:
            return
        points = stream['points'][begin:begin + count]
        final = int(begin + count == len(stream['points'])) + (2 if stream.get('transit', False) else 0)
        body = f"{sequence} {final} " + ','.join(points)
        crc = binascii.crc_hqx(body.encode('ascii'), 0xffff)
        payload = f'STR {self.active_id} {crc:04x} {body}\n'.encode('ascii')
        if len(payload) >= 2048:
            raise ValueError('stream packet exceeds firmware RX capacity')
        # Initial buffer must cover serial refill latency. Reject before motion.
        if begin == 0 and not (final & 1) and int(points[-1].split()[-1]) < 1000:
            raise ValueError('trajectory samples too dense for streaming buffer')
        stream['offset'] += count
        stream['sequence'] += 1
        if getattr(self, 'tx', None) is not None:
            raise ValueError('overlapping serial packets')
        self.tx = dict(payload=payload, sent=0, acked=0, last_progress=time.monotonic())
        self._pump_tx()

    def _pump_tx(self):
        """At most 32 unconsumed bytes; never interleave PING into a packet."""
        tx = getattr(self, 'tx', None)
        if tx is None:
            return
        now = time.monotonic()
        if tx['acked'] != tx['sent']:
            if now - tx['last_progress'] > 0.75:
                self.ser.write(b'\nSTOP\n')
                self._fail('RX_ACK_TIMEOUT')
            return
        start = tx['sent']
        block = tx['payload'][start:start + 32]
        if self.ser.write(block) != len(block):
            raise serial.SerialException('short stream write')
        tx['sent'] += len(block)
        tx['last_progress'] = now
        if tx['sent'] == len(tx['payload']):
            self.tx = None  # Newline completes this packet; await BUF3 credit.


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='auto')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--rate', type=float, default=100.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ArduinoBridge(args.port, args.baud, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.ser:
            node.ser.write(b'\nSTOP\n')
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():  # SIGTERM may already have shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
