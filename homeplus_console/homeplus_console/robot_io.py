"""The boundary between the console and the robot.

Both the signal-level sim (sim/world.py) and the rclpy layer (ros_io.py, M0/M1 on the
ROS machine) implement this interface. Everything above it (state machine, actions,
API, UI) is identical in both modes.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Protocol


class UnitStatus(Dict):
    """systemctl --user show fields: active_state, sub_state, pid, start_mono, restarts, result."""


class Supervisor(Protocol):
    units: List[str]

    def status(self) -> Dict[str, Dict]: ...
    def start(self, unit: str) -> Dict: ...
    def stop(self, unit: str) -> Dict: ...
    def restart(self, unit: str) -> Dict: ...


class RobotIO(Protocol):
    mode: str            # "sim" | "ros"
    structured: bool     # executor publishes /gesture_control/executor_state (§7.1)
    supervisor: Supervisor

    def publish_stop(self) -> bool: ...
    """Publish /arduino/stop True three times, 20 ms apart. False if nothing could carry it."""

    def call_cancel(self) -> Optional[bool]: ...
    """Call /gesture_control/cancel (§7.3). None when the service does not exist."""

    def send_command(self, command: str, token: str, args: Optional[Dict] = None) -> None: ...
    """Publish a GestureCommand on /gesture_control/command with a request token. Recovery commands
    (return_to_start, undo_step, undo_sequence, resume) need the §7.8–7.11 executor changes; until then
    ros_io composes return_to_start from gripper_open + home and reports undo/resume as unsupported."""

    def zero(self) -> Optional[bool]: ...
    """§7.10 ZERO via /arduino/zero. None when the firmware/bridge doesn't advertise zero_supported."""

    def request_reset(self) -> bool: ...
    """Reset-based re-zero: DTR toggle (laptop) or mac_agent restarting mac_serial_server.sh (Mac).
    False when the console can't trigger it and the operator must."""

    def send_approval(self, plan_id: Optional[int], approve: bool, operator: str) -> None: ...
    """Scoped approval (§7.3) when supported, else the Bool approval topic."""

    def clear_octomap(self) -> bool: ...
    """Clear the nvblox collision map (/nvblox/clear); the name is kept as the UI action id."""

    def external_instance(self, unit: str) -> List[str]: ...
    """Evidence (process, open device, ROS node) that the unit's component already runs outside the
    console. Checked fresh before every start/restart: the console must never start a duplicate."""

    def set_task_id(self, task_id: int) -> None: ...
