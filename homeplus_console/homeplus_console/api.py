"""FastAPI app: /ws full snapshots at 4 Hz, /api/actions/*, /img/*, static UI (UI_PLAN §1.2).

Every snapshot is a full state. A lost or reordered message can't leave the UI wrong;
the next snapshot corrects it.
"""
from __future__ import annotations

import asyncio
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .actions import ActionService, Audit
from .images import FrameStore
from .log_filter import LogBuffer
from . import robot_model
from .robot_io import RobotIO
from .signals import SignalStore
from .state_machine import FEEDS, Inputs, derive
from .tracker import Tracker

STATIC = Path(__file__).parent / "static"
SNAPSHOT_HZ = 4.0
DERIVE_HZ = 10.0


class Console:
    def __init__(self, io: RobotIO, store: SignalStore, frames: FrameStore, logs: LogBuffer,
                 tracker: Tracker, cfg: Dict[str, Any], data_dir: Path, profile: str):
        self.io, self.store, self.frames, self.logs, self.tracker = io, store, frames, logs, tracker
        self.cfg, self.profile = cfg, profile
        self.actions = ActionService(io, tracker, Audit(data_dir), cfg, logs)
        self._lock = threading.Lock()
        self.state: Optional[Dict[str, Any]] = None
        self.seq = 0
        self.link_paused_until = 0.0  # sim only: simulate a dropped console link
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._loop, name="derive", daemon=True).start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.step()
            except Exception as exc:
                self.logs.add("console", "ERROR", f"derive failed: {exc!r}")
            time.sleep(max(0.0, 1.0 / DERIVE_HZ - (time.monotonic() - t0)))

    def _follow_task(self, now: float) -> None:
        """The next task follows from the gripper's load: place (7) while holding the cup, otherwise
        not 7. With the load unknown (no executor status since the console started), accept the
        detector's current task if it is a startable one, so the preflight compares like with like."""
        from .actions import TASK_COMMANDS
        ex = self.store.get("/gesture_control/executor_state")
        holding = (ex.value or {}).get("holding_cup") if ex else None
        if holding is True:
            self.actions.requested_task = 7
        elif holding is False:
            if self.actions.requested_task == 7:
                self.actions.requested_task = 1
        elif (t := self.store.get("/molmopoint/task_id")) and t.fresh(now, 6.0) and t.value in TASK_COMMANDS:
            self.actions.requested_task = int(t.value)

    def step(self) -> Dict[str, Any]:
        now, wall = self.store.clock(), time.time()
        self.tracker.observe(now)
        self._follow_task(now)
        inputs = Inputs(
            signals=self.store.snapshot(), mem=self.tracker.mem, units=self.io.supervisor.status(),
            frozen_for={fid: self.frames.frozen_for(fid, now) for fid, *_ in FEEDS},
            frame_seq={fid: (f.seq if (f := self.frames.get(fid)) else None) for fid, *_ in FEEDS},
            logs_recent=self.logs.since(0, limit=300), cfg=self.cfg, structured=self.io.structured,
            mode=self.io.mode, flow=None, requested_task=self.actions.requested_task,
            foreign_publishers=(fp.value if (fp := self.store.get("graph/foreign")) and fp.fresh(now, 5.0) else []),
            external=(ex.value if (ex := self.store.get("graph/external")) and ex.fresh(now, 6.0) else {}),
            graph_nodes=(gn.value if (gn := self.store.get("graph/nodes")) and gn.fresh(now, 6.0) else {}))
        state = derive(inputs, now, wall)
        onsets = self.tracker.mem.state_onsets
        if state["robot"]["state"] not in onsets:
            onsets.clear()
            onsets[state["robot"]["state"]] = wall
            self.logs.add("console", "INFO", f"Robot state → {state['robot']['title']}: {state['robot']['detail']}")
        state["robot"]["since_wall"] = onsets[state["robot"]["state"]]
        extra = self.actions.tick(state, now)
        state.update(extra)
        state["stop"] = self.actions.stop_view()
        state["tracked_actions"] = self.actions.tracked_view()
        state["audit"] = list(self.actions.audit.recent)[-12:]
        state["logs"] = self.logs.since(0, limit=120)
        state["units"] = self.io.supervisor.units
        state["mode"] = self.io.mode
        state["profile"] = self.profile
        state["sim"] = self.io.fault_state() if self.io.mode == "sim" else None
        with self._lock:
            self.state = state
        return state

    def snapshot_for(self, client: Optional[str]) -> Dict[str, Any]:
        with self._lock:
            state = self.state
        if state is None:
            state = self.step()
        now = self.store.clock()
        self.seq += 1
        lease = self.actions.lease
        lease.valid(now)
        return {**state, "seq": self.seq, "server_wall": time.time(),
                "allowed": self.actions.allowed(state, client, now),
                "control": {"holder": lease.label, "held": lease.holder is not None,
                            "you": client is not None and lease.holder == client}}


def create_app(console: Console) -> FastAPI:
    app = FastAPI(title="HomePlus operator console")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        client: Optional[str] = None
        label = "operator"

        async def reader():
            nonlocal client, label
            while True:
                msg = await sock.receive_json()
                if msg.get("type") == "hello":
                    client = msg.get("client_id")
                    label = msg.get("label") or label
                if client:
                    console.actions.lease.beat(client, console.store.clock())

        task = asyncio.create_task(reader())
        try:
            while True:
                if time.time() >= console.link_paused_until:
                    await sock.send_json(console.snapshot_for(client))
                await asyncio.sleep(1.0 / SNAPSHOT_HZ)
                if task.done():
                    break
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()

    @app.post("/api/actions/{name}")
    async def action(name: str, request: Request):
        body = await request.json()
        rid = body.get("request_id")
        if not rid:
            raise HTTPException(400, "request_id required")
        with console._lock:
            state = console.state
        if state is None:
            raise HTTPException(503, "state not derived yet")
        res = console.actions.handle(name, rid, body.get("client_id"), body.get("operator") or "operator",
                                     body.get("args") or {}, state, console.store.clock())
        return JSONResponse(res)

    @app.get("/img/{feed}.jpg")
    async def img(feed: str):
        f = console.frames.get(feed)
        if f is None:
            raise HTTPException(404, "no frame")
        return Response(f.jpeg, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store", "X-Frame-Seq": str(f.seq)})

    @app.get("/api/robot/urdf")
    async def urdf():
        xml = robot_model.urdf_xml()
        if xml is None:
            raise HTTPException(404, "homeplus_urdf_description not found (set HOMEPLUS_URDF_DIR)")
        return Response(xml, media_type="application/xml", headers={"Cache-Control": "max-age=3600"})

    @app.get("/api/robot/pkg/meshes/{name}")
    async def mesh(name: str):
        p = robot_model.mesh_path(name)
        if p is None:
            raise HTTPException(404, "no such mesh")
        return FileResponse(p, media_type="model/stl", headers={"Cache-Control": "max-age=3600"})

    @app.get("/api/logs")
    async def logs(since: int = 0, important: bool = False):
        return console.logs.since(since, important_only=important, limit=2000)

    @app.get("/api/sim")
    async def sim_state():
        if console.io.mode != "sim":
            raise HTTPException(404, "not in sim mode")
        return console.io.fault_state()

    @app.post("/api/sim/fault")
    async def sim_fault(request: Request):
        if console.io.mode != "sim":
            raise HTTPException(404, "not in sim mode")
        body = await request.json()
        if body.get("name") == "link_pause":
            console.link_paused_until = time.time() + float(body.get("value", 5))
            return {"ok": True}
        return console.io.set_fault(body["name"], body.get("value"))

    if (STATIC / "index.html").exists():
        app.mount("/assets", StaticFiles(directory=STATIC / "assets"), name="assets")

        @app.get("/")
        async def index():
            return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})

    return app
