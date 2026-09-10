#!/usr/bin/env python3
"""
Bidirectional Arduino bridge: reads joint states from serial → /joint_states,
and writes commands from /arduino/command_queue → serial.

To run manually:
  python3 arduino_reader.py --port auto --baud 115200 --rate 20
"""

import argparse
import glob
import math
import os
import re
import time
from typing import List, Optional

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool


JOINT_NAMES = [
    "base_x",
    "base_y",
    "base_theta",
    "joint_grip_L",
    "joint_hand",
    "joint_wrist",
    "joint_elbow",
    "joint_shoulder",
]

# The Arduino protocol does not report wheel rotation, but robot_state_publisher
# still needs values for these non-fixed URDF joints to publish the wheel TFs.
WHEEL_JOINT_NAMES = [
    "joint_wheel_FL",
    "joint_wheel_FR",
    "joint_wheel_RL",
    "joint_wheel_RR",
]

# AVR boards have a small hardware receive ring. Sending a complete MoveIt
# trajectory as one uninterrupted burst can occasionally drop a comma or
# space even though pyserial reports a successful write. The firmware then
# correctly rejects the damaged line with ERROR INCORRECT_INPUT_COUNT.
SERIAL_TX_CHUNK_BYTES = 16
SERIAL_TX_INTER_CHUNK_DELAY_SEC = 0.010
SERIAL_PARSE_RETRY_DELAY_SEC = 0.100
SERIAL_PARSE_MAX_RETRIES = 2
FINAL_SETTLE_RETRY_DELAY_SEC = 0.100
FINAL_SETTLE_MAX_RETRIES = 1
COMMAND_VALUE_COUNT = 10

# A final goal this close is safe to command once more after the firmware's
# five-second settle timeout. Larger residuals remain hard failures.
NEAR_GOAL_TIMEOUT_LIMITS = {
    "dx_cm": 0.5,
    "dy_cm": 0.5,
    "dth_deg": 0.5,
    "grip_us": 3.0,
    "hand_us": 3.0,
    "wrist_us": 3.0,
    "elbow_deg": 1.0,
    "shoulder_deg": 2.5,
}

# If the one final-waypoint retry cannot overcome wheel/servo deadband, accept
# only a tighter residual as a soft completion. This prevents a persistent
# 2-3 mm base error at an otherwise reached pose from aborting a complete task,
# while preserving hard failures for meaningful position errors.
FINAL_SETTLE_ACCEPT_LIMITS = {
    "dx_cm": 0.35,
    "dy_cm": 0.35,
    "dth_deg": 0.25,
    "grip_us": 2.1,
    "hand_us": 2.1,
    "wrist_us": 2.1,
    "elbow_deg": 1.1,
    "shoulder_deg": 2.6,
}


class ArduinoBridge(Node):
    def __init__(
        self,
        port: str = "auto",
        baud: int = 115200,
        joint_names: Optional[List[str]] = None,
        read_hz: float = 20.0,
    ):
        super().__init__("arduino_bridge")
        self.port = port
        self.active_port: Optional[str] = None
        self.baud = int(baud)
        self.read_hz = float(read_hz)
        self.joint_names = joint_names or list(JOINT_NAMES)
        self.current_positions = [0.0] * len(self.joint_names)
        self._pending_command_positions: Optional[List[float]] = None

        # --- Publishers ---
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ack_pub = self.create_publisher(String, "/arduino/ack", 10)
        self.status_pub = self.create_publisher(Bool, "/arduino/status", 10)

        # --- Subscribers ---
        self.create_subscription(
            String, "/arduino/command_queue", self._command_queue_callback, 10
        )

        self.ser: Optional[serial.Serial] = None
        self._last_warned_line = ""
        # Retain a queue only until the firmware confirms that it parsed the
        # complete line. ERROR INCORRECT_INPUT_COUNT is safe to retry because
        # the firmware validates every waypoint before starting any motion.
        self._pending_serial_payloads: List[str] = []
        self._pending_serial_waypoint_count = 0
        self._serial_parse_retry_count = 0
        self._pending_final_waypoint: Optional[str] = None
        self._final_settle_retry_count = 0

        self.get_logger().info(
            f"ArduinoBridge init: port={self.port} baud={self.baud} rate={self.read_hz} Hz "
            f"joints={len(self.joint_names)}"
        )

        self._open_serial()
        period = 1.0 / max(1.0, self.read_hz)
        self.timer = self.create_timer(period, self._read_loop)

    @staticmethod
    def _candidate_ports() -> List[str]:
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        tty_ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        # Prefer persistent by-id names; keep insertion order while removing duplicates.
        return list(dict.fromkeys(by_id + tty_ports))

    def _resolve_port(self) -> Optional[str]:
        if self.port and self.port.lower() != "auto":
            return self.port

        candidates = self._candidate_ports()
        if not candidates:
            self.get_logger().error(
                "No Arduino serial device found. Checked /dev/serial/by-id/*, "
                "/dev/ttyACM*, and /dev/ttyUSB*."
            )
            return None

        selected = candidates[0]
        if selected.startswith("/dev/serial/by-id/"):
            target = os.path.realpath(selected)
            self.get_logger().info(f"Auto-selected Arduino serial {selected} -> {target}")
        else:
            self.get_logger().info(f"Auto-selected Arduino serial {selected}")
        return selected

    def _publish_joint_state(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names) + WHEEL_JOINT_NAMES
        js.position = list(self.current_positions) + [0.0] * len(WHEEL_JOINT_NAMES)
        self.joint_state_pub.publish(js)

    # ------------------------------------------------------------------
    # Serial management
    # ------------------------------------------------------------------
    def _open_serial(self) -> None:
        port = self._resolve_port()
        if port is None:
            self.ser = None
            return

        try:
            # Keep this well below the ROS timer period. A one-second blocking
            # readline starves command subscriptions because this node uses a
            # single-threaded executor, producing multi-second send latency.
            self.ser = serial.Serial(port, self.baud, timeout=0.02)
            self.active_port = port
            time.sleep(2.0)
            self.get_logger().info(f"Opened serial {self.active_port}")
        except Exception as exc:
            self.get_logger().error(f"Failed to open serial {port}: {exc}")
            self.active_port = None
            self.ser = None

    def _ensure_serial(self) -> bool:
        if self.ser is not None and getattr(self.ser, "is_open", False):
            return True
        self._open_serial()
        return self.ser is not None and getattr(self.ser, "is_open", False)

    # ------------------------------------------------------------------
    # Read: Arduino → /joint_states
    # ------------------------------------------------------------------
    def _read_loop(self) -> None:
        if not self._ensure_serial():
            self._publish_joint_state()
            return

        try:
            raw = self.ser.readline()
            if not raw:
                self._publish_joint_state()
                return

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                self._publish_joint_state()
                return

            # Status/ack lines from Arduino — not joint data
            lower = line.lower()
            if lower.startswith("done"):
                # The firmware reports completion but not necessarily joint
                # feedback.  Promote the final commanded waypoint so
                # /joint_states stays at the reached pose instead of zeros.
                if self._pending_command_positions is not None:
                    self.current_positions = self._pending_command_positions
                    self._pending_command_positions = None
                self._publish_joint_state()
                # Publish that Arduino has reached finished state
                self.ack_pub.publish(String(data=line))
                done_msg = Bool(data=True)
                self.status_pub.publish(done_msg)
                self._clear_pending_serial_queue()
                self._clear_pending_command_tracking()
                self.get_logger().info(f"Arduino status: {line}")
                return
            if lower.startswith("error incorrect_input_count"):
                if (
                    self._pending_serial_payloads
                    and self._serial_parse_retry_count < SERIAL_PARSE_MAX_RETRIES
                ):
                    self._serial_parse_retry_count += 1
                    self.get_logger().warning(
                        "Arduino rejected a damaged serial queue; resending the "
                        f"same validated payload (retry "
                        f"{self._serial_parse_retry_count}/{SERIAL_PARSE_MAX_RETRIES})"
                    )
                    time.sleep(SERIAL_PARSE_RETRY_DELAY_SEC)
                    self._write_serial_payloads(
                        self._pending_serial_payloads,
                        self._pending_serial_waypoint_count,
                        retry=True,
                    )
                    self._publish_joint_state()
                    return

                self._clear_pending_serial_queue()
                self._clear_pending_command_tracking()
            if lower.startswith("error timeout"):
                near_goal = self._is_timeout_within_limits(
                    line, NEAR_GOAL_TIMEOUT_LIMITS
                )
                if (
                    self._pending_final_waypoint is not None
                    and self._final_settle_retry_count < FINAL_SETTLE_MAX_RETRIES
                    and near_goal
                ):
                    self._final_settle_retry_count += 1
                    final_waypoint = self._pending_final_waypoint
                    self.get_logger().warning(
                        "Arduino timed out within the near-goal settling limits; "
                        f"resending only the final waypoint (retry "
                        f"{self._final_settle_retry_count}/"
                        f"{FINAL_SETTLE_MAX_RETRIES}): {line}"
                    )
                    self._pending_serial_payloads = [final_waypoint]
                    self._pending_serial_waypoint_count = 1
                    self._serial_parse_retry_count = 0
                    time.sleep(FINAL_SETTLE_RETRY_DELAY_SEC)
                    self._write_serial_payloads(
                        [final_waypoint],
                        1,
                        retry=True,
                    )
                    self._publish_joint_state()
                    return
                if (
                    self._pending_final_waypoint is not None
                    and self._final_settle_retry_count >= FINAL_SETTLE_MAX_RETRIES
                    and self._is_timeout_within_limits(
                        line, FINAL_SETTLE_ACCEPT_LIMITS
                    )
                ):
                    # The firmware reports target-minus-current residuals but
                    # does not stream a complete measured joint state. Match
                    # normal DONE handling and promote the final commanded
                    # waypoint so TF and the next plan remain synchronized.
                    if self._pending_command_positions is not None:
                        self.current_positions = self._pending_command_positions
                        self._pending_command_positions = None
                    self._publish_joint_state()
                    soft_done = f"DONE NEAR_GOAL_AFTER_RETRY {line}"
                    self.ack_pub.publish(String(data=soft_done))
                    self.status_pub.publish(Bool(data=True))
                    self._clear_pending_serial_queue()
                    self._clear_pending_command_tracking()
                    self.get_logger().warning(
                        "Accepting final waypoint within tight near-goal "
                        f"limits after retry: {line}"
                    )
                    return
            if (lower.startswith("proceed")
                    or lower.startswith("timeout")
                    or lower.startswith("ack")
                    or lower.startswith("ok")
                    or lower.startswith("done")
                    or lower.startswith("error")
                    or lower.startswith("usb serial")):
                if lower.startswith("ack queued"):
                    # Parsing succeeded and motion has started. A later retry
                    # would be unsafe because it could duplicate execution.
                    self._clear_pending_serial_queue()
                elif lower.startswith("error"):
                    self._clear_pending_serial_queue()
                    self._clear_pending_command_tracking()
                self.ack_pub.publish(String(data=line))
                self.get_logger().info(f"Arduino: {line}")
                self._publish_joint_state()
                return

            parts = [p.strip() for p in line.replace(",", " ").split() if p.strip()]
            if len(parts) < len(self.joint_names):
                if line != self._last_warned_line:
                    self.get_logger().warning(
                        f"Got {len(parts)} values, expected {len(self.joint_names)}: '{line}'"
                    )
                    self._last_warned_line = line
                self._publish_joint_state()
                return

            positions = self._command_to_joint_positions(line)
            if positions is None:
                self.get_logger().warning(
                    f"Could not convert Arduino state to ROS units: '{line}'"
                )
                self._publish_joint_state()
                return

            self.current_positions = positions
            self._publish_joint_state()
        except Exception as exc:
            self.get_logger().error(f"Error reading serial: {exc}")
            self._publish_joint_state()

    # ------------------------------------------------------------------
    # Write: /arduino/command_queue → Arduino
    # ------------------------------------------------------------------
    @staticmethod
    def _split_command_waypoints(command: str) -> List[str]:
        lines = [line.strip() for line in command.splitlines() if line.strip()]
        if len(lines) > 1:
            return lines
        if len(lines) == 1 and "," in lines[0]:
            return [part.strip() for part in lines[0].split(",") if part.strip()]
        return lines or ([command] if command else [])

    @staticmethod
    def _command_to_joint_positions(command: str) -> Optional[List[float]]:
        """Convert one Arduino waypoint into ROS joint-state units."""
        parts = command.replace(",", " ").split()
        if len(parts) < len(JOINT_NAMES):
            return None
        try:
            values = [float(value) for value in parts[:len(JOINT_NAMES)]]
        except ValueError:
            return None

        return [
            values[0] / 100.0,
            values[1] / 100.0,
            math.radians(values[2]),
            math.radians(values[3]),
            math.radians(values[4]),
            math.radians(values[5]),
            math.radians(values[6]),
            math.radians(values[7]),
        ]

    @staticmethod
    def _validate_waypoints(waypoints: List[str]) -> Optional[str]:
        """Return an error description, or None when firmware will accept them."""
        for index, waypoint in enumerate(waypoints):
            fields = waypoint.split()
            if len(fields) != COMMAND_VALUE_COUNT:
                return (
                    f"waypoint {index} has {len(fields)} values; "
                    f"expected {COMMAND_VALUE_COUNT}"
                )
            try:
                values = [float(field) for field in fields]
            except ValueError as exc:
                return f"waypoint {index} contains a non-numeric value: {exc}"
            if not all(math.isfinite(value) for value in values):
                return f"waypoint {index} contains a non-finite value"
        return None

    def _clear_pending_serial_queue(self) -> None:
        self._pending_serial_payloads = []
        self._pending_serial_waypoint_count = 0
        self._serial_parse_retry_count = 0

    def _clear_pending_command_tracking(self) -> None:
        self._pending_command_positions = None
        self._pending_final_waypoint = None
        self._final_settle_retry_count = 0

    @staticmethod
    def _is_timeout_within_limits(line: str, limits) -> bool:
        metrics = {
            name: abs(float(value))
            for name, value in re.findall(
                r"(dx_cm|dy_cm|dth_deg|grip_us|hand_us|wrist_us|"
                r"elbow_deg|shoulder_deg)=([-+]?\d+(?:\.\d+)?)",
                line,
            )
        }
        if set(metrics) != set(limits):
            return False
        return all(
            metrics[name] <= limit
            for name, limit in limits.items()
        )

    @staticmethod
    def _is_near_goal_timeout(line: str) -> bool:
        return ArduinoBridge._is_timeout_within_limits(
            line, NEAR_GOAL_TIMEOUT_LIMITS
        )

    def _write_serial_payloads(
        self,
        serial_payloads: List[str],
        waypoint_count: int,
        retry: bool = False,
    ) -> None:
        if self.ser is None:
            raise serial.SerialException("serial port is not open")

        action = "Resending" if retry else "Sending"
        for index, payload in enumerate(serial_payloads, start=1):
            self.get_logger().info(
                f"  {action} Arduino serial payload {index}/{len(serial_payloads)} "
                f"({len(payload)} chars, paced {SERIAL_TX_CHUNK_BYTES}-byte chunks)"
            )
            encoded = f"{payload}\r\n".encode()
            for offset in range(0, len(encoded), SERIAL_TX_CHUNK_BYTES):
                chunk = encoded[offset:offset + SERIAL_TX_CHUNK_BYTES]
                written = self.ser.write(chunk)
                if written != len(chunk):
                    raise serial.SerialException(
                        f"Short serial write: wrote {written}/{len(chunk)} bytes"
                    )
                # tcdrain/flush makes the pause occur after this chunk has
                # physically left the host, rather than merely after it
                # entered the operating-system transmit buffer.
                self.ser.flush()
                if offset + SERIAL_TX_CHUNK_BYTES < len(encoded):
                    time.sleep(SERIAL_TX_INTER_CHUNK_DELAY_SEC)
            time.sleep(0.02)

        verb = "Resent" if retry else "Finished writing"
        self.get_logger().info(
            f"{verb} queue containing {waypoint_count} waypoint(s) "
            "to Arduino serial"
        )

    def _command_queue_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return

        if not self._ensure_serial():
            self.get_logger().error("Cannot send to Arduino: serial not open")
            return

        try:
            waypoints = self._split_command_waypoints(command)
            if not waypoints:
                return

            validation_error = self._validate_waypoints(waypoints)
            if validation_error is not None:
                error = f"ERROR HOST_INVALID_COMMAND: {validation_error}"
                self.get_logger().error(error)
                self.ack_pub.publish(String(data=error))
                return

            # False means hardware motion is in progress. Moving-camera cloud
            # builders use this to pause OctoMap insertion until the final
            # commanded joint state is published on DONE.
            self.status_pub.publish(Bool(data=False))

            # Preserve a comma-delimited queue as one serial line. The HomePlus
            # firmware only calls parseCommandQueue() once it receives a newline;
            # sending every comma-separated waypoint as its own line resets the
            # firmware queue on every waypoint. Newline mode remains available
            # for firmware that explicitly supports incremental enqueueing.
            newline_payloads = [
                line.strip() for line in command.splitlines() if line.strip()
            ]
            serial_payloads = (
                [command]
                if len(newline_payloads) == 1 and "," in command
                else newline_payloads
            )

            self.get_logger().info(
                f"Arduino command queue received: {len(waypoints)} waypoint(s), "
                f"{len(command)} chars, {len(serial_payloads)} serial payload(s)"
            )

            self._pending_serial_payloads = list(serial_payloads)
            self._pending_serial_waypoint_count = len(waypoints)
            self._serial_parse_retry_count = 0
            self._pending_final_waypoint = waypoints[-1]
            self._final_settle_retry_count = 0
            self._write_serial_payloads(serial_payloads, len(waypoints))

            final_command_positions = self._command_to_joint_positions(waypoints[-1])
            if final_command_positions is not None:
                self._pending_command_positions = final_command_positions

        except serial.SerialException as exc:
            self._clear_pending_serial_queue()
            self._clear_pending_command_tracking()
            self.get_logger().error(f"Serial write error: {exc}")
        except Exception as exc:
            self._clear_pending_serial_queue()
            self._clear_pending_command_tracking()
            self.get_logger().error(f"Unexpected error writing to Arduino: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", default="115200")
    parser.add_argument("--rate", default="20")
    parser.add_argument("--joint-names", default="")
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()

    joint_names = None
    if args.joint_names:
        joint_names = [n.strip() for n in args.joint_names.split(",") if n.strip()]

    rclpy.init()
    node = ArduinoBridge(
        port=args.port,
        baud=int(args.baud),
        joint_names=joint_names,
        read_hz=float(args.rate),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and getattr(node.ser, "is_open", False):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
