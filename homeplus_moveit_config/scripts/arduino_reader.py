#!/usr/bin/env python3
"""
Bidirectional Arduino bridge: reads joint states from serial → /joint_states,
and writes commands from /arduino/command_queue → serial.

To run manually:
  python3 arduino_reader.py --port /dev/ttyUSB0 --baud 9600 --rate 20
"""

import argparse
import time
from typing import List, Optional

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


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
        port: str = "/dev/ttyUSB0",
        baud: int = 9600,
        joint_names: Optional[List[str]] = None,
        read_hz: float = 20.0,
    ):
        super().__init__("arduino_bridge")
        self.port = port
        self.baud = int(baud)
        self.read_hz = float(read_hz)
        self.joint_names = joint_names or list(JOINT_NAMES)

        # --- Publishers ---
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ack_pub = self.create_publisher(String, "/arduino/ack", 10)

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

    # ------------------------------------------------------------------
    # Serial management
    # ------------------------------------------------------------------
    def _open_serial(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            self.get_logger().info(f"Opened serial {self.port}")
        except Exception as exc:
            self.get_logger().error(f"Failed to open serial {self.port}: {exc}")
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
            return

        try:
            raw = self.ser.readline()
            if not raw:
                return

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                return

            # Status/ack lines from Arduino — not joint data
            if (line.lower().startswith("proceed")
                    or line.lower().startswith("ack")
                    or line.lower().startswith("ok")
                    or line.lower().startswith("done")
                    or line.lower().startswith("error")
                    or line.lower().startswith("usb serial")):
                self.ack_pub.publish(String(data=line))
                self.get_logger().info(f"Arduino: {line}")
                return

            parts = [p.strip() for p in line.replace(",", " ").split() if p.strip()]
            if len(parts) < len(self.joint_names):
                if line != self._last_warned_line:
                    self.get_logger().warning(
                        f"Got {len(parts)} values, expected {len(self.joint_names)}: '{line}'"
                    )
                    self._last_warned_line = line
                return

            positions = []
            for i in range(len(self.joint_names)):
                try:
                    positions.append(float(parts[i]))
                except ValueError:
                    positions.append(0.0)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.joint_names)
            js.position = positions
            self.joint_state_pub.publish(js)
        except Exception as exc:
            self.get_logger().error(f"Error reading serial: {exc}")

    # ------------------------------------------------------------------
    # Write: /arduino/command_queue → Arduino
    # ------------------------------------------------------------------
    def _command_queue_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return

        if not self._ensure_serial():
            self.get_logger().error("Cannot send to Arduino: serial not open")
            return

        try:
            self.ser.write(f"{command}\n".encode())
            self.get_logger().info(
                f"Sent to Arduino ({len(command)} chars): {command[:80]}{'...' if len(command) > 80 else ''}"
            )
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write error: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected error writing to Arduino: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", default="/dev/ttyUSB0")
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
