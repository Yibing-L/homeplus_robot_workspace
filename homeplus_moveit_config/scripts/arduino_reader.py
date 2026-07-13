#!/usr/bin/env python3
"""
Bidirectional Arduino bridge: reads joint states from serial → /joint_states,
and writes commands from /arduino/command_queue → serial.

To run manually:
  python3 arduino_reader.py --port auto --baud 9600 --rate 20
"""

import argparse
import glob
import os
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


class ArduinoBridge(Node):
    def __init__(
        self,
        port: str = "auto",
        baud: int = 9600,
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
        js.name = list(self.joint_names)
        js.position = list(self.current_positions)
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
            self.ser = serial.Serial(port, self.baud, timeout=1)
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
                # Publish that Arduino has reached finished state
                self.ack_pub.publish(String(data=line))
                done_msg = Bool(data=True)
                self.status_pub.publish(done_msg)
                self.get_logger().info(f"Arduino status: {line}")
                self._publish_joint_state()
                return
            if (lower.startswith("proceed")
                    or lower.startswith("timeout")
                    or lower.startswith("ack")
                    or lower.startswith("ok")
                    or lower.startswith("done")
                    or lower.startswith("error")
                    or lower.startswith("usb serial")):
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

            positions = []
            for i in range(len(self.joint_names)):
                try:
                    positions.append(float(parts[i]))
                except ValueError:
                    positions.append(0.0)

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

    def _command_queue_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return

        if not self._ensure_serial():
            self.get_logger().error("Cannot send to Arduino: serial not open")
            return

        try:
            lines = self._split_command_waypoints(command)
            if not lines:
                return

            self.get_logger().info(
                f"Arduino command queue received: {len(lines)} waypoint(s), "
                f"{len(command)} chars"
            )

            for index, line in enumerate(lines, start=1):
                self.get_logger().info(
                    f"  Arduino send {index}/{len(lines)}: {line}"
                )
                self.ser.write(f"{line}\r\n".encode())
                self.ser.flush()
                time.sleep(0.02)

            self.get_logger().info(
                f"Finished writing {len(lines)} waypoint(s) to Arduino serial"
            )
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write error: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected error writing to Arduino: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", default="9600")
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
