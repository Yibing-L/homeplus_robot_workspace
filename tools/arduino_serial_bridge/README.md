# Arduino serial bridge (Mac -> ROS VM)

UTM USB passthrough drops the Arduino and re-enumerates it, which also resets
the board and re-zeroes odometry. This bridge keeps the Arduino on the Mac and
forwards its serial port to the VM over the UTM shared network.

Mac (keeps running, restarts if the Arduino is unplugged):

    tools/arduino_serial_bridge/mac_serial_server.sh            # 192.168.64.1:8767

VM (creates ~/ttyArduino, reconnects automatically):

    tools/arduino_serial_bridge/vm_serial_client.sh
    ros2 launch homeplus_moveit_config gesture_control.launch.py arduino_port:=$HOME/ttyArduino ...

Notes:
- Requires `socat` on both sides (`brew install socat`, `apt install socat`).
- macOS socat needs `ispeed/ospeed` (not `b115200`) and flow control off
  (`crtscts=0,ixon=0,ixoff=0`); with flow control on, the stream stalls after
  the first host write.
- Opening the port on the Mac (restarting `mac_serial_server.sh`) resets the
  Arduino and therefore zeroes odometry where the robot stands. Restarting the
  ROS launch on the VM side does not.
- Do not attach the Arduino to the VM in UTM while the bridge is in use.
