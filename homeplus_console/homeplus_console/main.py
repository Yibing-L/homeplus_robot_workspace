"""Entry point.

    python -m homeplus_console.main --sim                # signal-level sim, no ROS needed
    ros2 run homeplus_console console --profile mac_vm   # on the ROS machine (needs ros_io.py, M1 on hardware)
"""
from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

import uvicorn
import yaml

from . import config
from .api import Console, create_app
from .images import FrameStore
from .log_filter import LogBuffer
from .signals import SignalStore
from .tracker import Tracker


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="HomePlus operator console")
    ap.add_argument("--sim", action="store_true", help="signal-level simulation (no ROS)")
    ap.add_argument("--profile", default="mac_vm", choices=["mac_vm", "laptop"])
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; 127.0.0.1 by default, 192.168.64.2 for the Mac profile")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--data-dir", type=Path, default=None, help="audit log and origin record")
    ap.add_argument("--profile-file", type=Path, default=None,
                    help="profile YAML (default config/profiles/<profile>.yaml): units, serial_bridge, gpu_health_url")
    ap.add_argument("--config", type=Path, default=Path(__file__).resolve().parent.parent / "config" / "console.yaml")
    args = ap.parse_args(argv)

    cfg = config.load(args.config)
    profile_file = args.profile_file or Path(__file__).resolve().parent.parent / "config" / "profiles" / f"{args.profile}.yaml"
    prof = yaml.safe_load(profile_file.read_text()) if profile_file.exists() else {}
    cfg["remote_gpu"] = bool(prof.get("gpu_health_url")) if prof else args.profile == "mac_vm"
    # Mac profile: Arduino via tools/arduino_serial_bridge ($HOME/ttyArduino); a bridge restart doesn't reset it.
    cfg["serial_bridge"] = bool(prof.get("serial_bridge", args.profile == "mac_vm"))
    store, frames, logs = SignalStore(), FrameStore(), LogBuffer()
    if args.sim:
        from .sim_world import SimWorld
        data_dir = args.data_dir or Path(tempfile.mkdtemp(prefix="homeplus_console_sim_"))
        io = SimWorld(store, frames, logs, serial_bridge=cfg["serial_bridge"])
        tracker = Tracker(store, data_dir, preconfirm_origin=True)
    else:
        try:
            from .ros_io import RosIO
        except ImportError as exc:
            raise SystemExit(f"ROS mode is not available yet ({exc}). Run with --sim.")
        data_dir = args.data_dir or Path.home() / ".homeplus_console"
        io = RosIO(store, frames, logs, profile=args.profile, profile_path=profile_file)
        tracker = Tracker(store, data_dir)
    console = Console(io, store, frames, logs, tracker, cfg, data_dir, args.profile)
    io.start()
    console.start()
    print(f"HomePlus console ({'SIM' if args.sim else 'ROS'}, profile {args.profile}) on http://{args.host}:{args.port}  "
          f"data: {data_dir}")
    uvicorn.run(create_app(console), host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
