#!/usr/bin/env python3
"""
ROS node that reads lines from Arduino serial port and publishes on /joint_states.
To run manually for debugging: python3 /home/homeplus/Desktop/homeplus_robot_workspace-main/homeplus_moveit_config/scripts/arduino_reader.py --port /dev/ttyUSB0 --baud 9600 --rate 20
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import argparse


class ArduinoSerialReader(Node):
    def __init__(self, port='/dev/ttyUSB0', baud=9600, joint_names=None, read_hz=20.0):
        super().__init__('arduino_serial_reader')
        self.port = port
        self.baud = int(baud)
        self.read_hz = float(read_hz)

        if joint_names is None:
            self.joint_names = [
                'base_x', 'base_y', 'base_theta',
                'JointFingerL', 'JointHand', 'JointWrist',
                'JointArm2', 'JointArm1', 'JointTelescope', 'JointFrame'
            ]
        else:
            self.joint_names = joint_names

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info(f'ArduinoSerialReader init: port={self.port} baud={self.baud} rate={self.read_hz} Hz')

        # Open serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)  
            self.get_logger().info(f'Opened serial {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')
            self.ser = None

        # Read from serial periodically
        period = 1.0 / max(1.0, float(self.read_hz))
        self.timer = self.create_timer(period, self.read_loop)

    def read_loop(self):
        # If serial is not open, try to reopen 
        if self.ser is None or not getattr(self.ser, 'is_open', False):
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2.0)
                self.get_logger().info(f'Reopened serial {self.port}')
            except Exception:
                return

        try:
            raw = self.ser.readline()
            if not raw:
                return
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                return

            # Accept either comma or space separated values (TODO edit this according to format)
            if ',' in line:
                parts = [p.strip() for p in line.split(',') if p.strip()]
            else:
                parts = [p.strip() for p in line.split() if p.strip()]

            if len(parts) < len(self.joint_names):
                self.get_logger().warning(f'Got {len(parts)} values, expected {len(self.joint_names)}: "{line}"')
                return

            # Make joint state message
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            positions = []
            for i in range(len(self.joint_names)):
                try:
                    positions.append(float(parts[i]))
                except Exception:
                    positions.append(0.0)
            js.position = positions
            self.pub.publish(js)
            self.get_logger().info(f'Published joint states: {positions}')
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', default='9600')
    parser.add_argument('--rate', default='20')
    parser.add_argument('--joint-names', default='')
    args, _ = parser.parse_known_args()

    joint_names = None
    if args.joint_names:
        joint_names = [n.strip() for n in args.joint_names.split(',') if n.strip()]

    rclpy.init()
    node = ArduinoSerialReader(port=args.port, baud=args.baud, joint_names=joint_names, read_hz=float(args.rate))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and getattr(node.ser, 'is_open', False):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
