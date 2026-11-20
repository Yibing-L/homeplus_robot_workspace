import { useEffect, useRef, useState } from "react";
import { act, wsSend } from "../store";
import type { Snapshot, TeleopView } from "../types";
import { Plan3D } from "./Plan3D";

// Teleop: keyboard drives the base, mouse drags the gripper (see homeplus_console/TELEOP.md).
// The browser only reports what is held; the backend owns every limit and safety check.
// Input frames go out every 33 ms on the console WebSocket while teleop is on, this tab holds
// control, and the page is focused. The backend stops the robot when frames stop for 300 ms.

const SEND_MS = 33;

type Axis = { pos: string[]; neg: string[] };
const AXES: Record<string, Axis> = {
  fwd: { pos: ["KeyW"], neg: ["KeyS"] },
  strafe: { pos: ["KeyA"], neg: ["KeyD"] },
  turn: { pos: ["KeyQ"], neg: ["KeyE"] },
  pitch: { pos: ["KeyR"], neg: ["KeyF"] },
  roll: { pos: ["KeyX"], neg: ["KeyZ"] },
  grip: { pos: ["KeyC"], neg: ["KeyO"] },
};
// Single-joint mode (2026-11-15): J toggles; 1-4 pick the joint the mouse/wheel turns.
const JOINTS: [string, string, string][] = [
  ["Digit1", "joint_shoulder", "shoulder"], ["Digit2", "joint_elbow", "elbow"],
  ["Digit3", "joint_hand", "wrist pitch"], ["Digit4", "joint_wrist", "wrist roll"],
];
const HANDLED = new Set([...Object.values(AXES).flatMap((a) => [...a.pos, ...a.neg]), "KeyG", "KeyJ", "Space", "ShiftLeft", "ShiftRight", ...JOINTS.map((j) => j[0])]);

const SENS_KEY = "homeplus.teleop.sensitivity";
let sens = 2.5;
try { const v = Number(localStorage.getItem(SENS_KEY)); if (v >= 0.5 && v <= 10) sens = v; } catch { /* storage blocked */ }
const SPEED_KEY = "homeplus.teleop.armSpeedCm";
let armSpeedCm = 4;
try { const v = Number(localStorage.getItem(SPEED_KEY)); if (v >= 2 && v <= 10) armSpeedCm = v; } catch { /* storage blocked */ }
const armSpeed = () => armSpeedCm / 100;
const sensitivity = () => sens;
function setSensitivity(v: number) {
  sens = v;
  try { localStorage.setItem(SENS_KEY, String(v)); } catch { /* storage blocked */ }
}

function SensitivitySlider() {
  const [v, setV] = useState(sens);
  const [sp, setSp] = useState(armSpeedCm);
  return (
    <>
      <label className="tsens" title="How far the gripper target moves per mouse movement / wheel notch">
        arm sensitivity
        <input type="range" min={0.5} max={10} step={0.25} value={v}
          onChange={(e) => { const n = Number(e.target.value); setV(n); setSensitivity(n); }} />
        <span className="mono">{v.toFixed(2)}×</span>
      </label>
      <label className="tsens" title="Top gripper speed (Shift adds 50%). Higher speeds trip the firmware's 3° tracking limit more often.">
        arm speed
        <input type="range" min={2} max={10} step={0.5} value={sp}
          onChange={(e) => { const n = Number(e.target.value); setSp(n); armSpeedCm = n; try { localStorage.setItem(SPEED_KEY, String(n)); } catch { /* blocked */ } }} />
        <span className="mono">{sp.toFixed(1)} cm/s</span>
      </label>
    </>
  );
}

function typing(t: EventTarget | null): boolean {
  const el = t as HTMLElement | null;
  return !!el && (el.tagName === "INPUT" || el.tagName === "TEXTAREA" || el.tagName === "SELECT" || el.isContentEditable);
}

/** Captures keys and mouse while `on`; returns the held keys and a ref-callback for the drag area. */
function useTeleopInput(on: boolean) {
  const keys = useRef(new Set<string>());
  const [shown, setShown] = useState<string[]>([]);
  // drag = a button physically held, or (mouse grabbed) movement/scroll in the last 300 ms.
  // 2026-11-15: pointer lock used to count as "held" forever, so a HOLD latch never cleared.
  const mouse = useRef({ dx: 0, dy: 0, wheel: 0, drag: false, toggles: 0, last: 0 });
  const [jointMode, setJointMode] = useState(false);
  const [joint, setJoint] = useState(1);   // index into JOINTS (elbow)
  const armJoint = useRef<string | null>(null);
  armJoint.current = jointMode ? JOINTS[joint][1] : null;
  const seq = useRef(0);
  const area = useRef<HTMLDivElement | null>(null);
  const [grabbed, setGrabbed] = useState(false);

  useEffect(() => {
    if (!on) {
      keys.current.clear();
      mouse.current = { dx: 0, dy: 0, wheel: 0, drag: false, toggles: 0, last: 0 };
      setShown([]);
      return;
    }
    const sync = () => setShown([...keys.current].sort());
    const down = (e: KeyboardEvent) => {
      if (typing(e.target) || e.ctrlKey || e.metaKey || e.altKey || !HANDLED.has(e.code)) return;
      e.preventDefault();
      if (e.code === "Space") {
        if (!e.repeat) void act("stop");
        return;
      }
      if (e.code === "KeyG" && !e.repeat) mouse.current.toggles += 1;
      if (e.code === "KeyJ") { if (!e.repeat) setJointMode((v) => !v); return; }
      const ji = JOINTS.findIndex((j) => j[0] === e.code);
      if (ji >= 0) { setJoint(ji); setJointMode(true); return; }
      keys.current.add(e.code);
      sync();
    };
    const up = (e: KeyboardEvent) => {
      if (keys.current.delete(e.code)) sync();
    };
    const release = () => {
      keys.current.clear();
      mouse.current.drag = false;
      sync();
    };
    window.addEventListener("keydown", down, true);
    window.addEventListener("keyup", up, true);
    window.addEventListener("blur", release);
    document.addEventListener("visibilitychange", release);
    const timer = window.setInterval(() => {
      const k = keys.current;
      const ax = (a: Axis) => (a.pos.some((c) => k.has(c)) ? 1 : 0) - (a.neg.some((c) => k.has(c)) ? 1 : 0);
      const m = mouse.current;
      const intent: Record<string, unknown> = Object.fromEntries(Object.entries(AXES).map(([n, a]) => [n, ax(a)]));
      intent.fast = k.has("ShiftLeft") || k.has("ShiftRight");
      intent.drag = m.drag || performance.now() - m.last < 300;
      intent.keys = [...k].map((c) => c.replace(/^Key/, "").replace(/Left|Right/, ""));
      seq.current += 1;
      wsSend({
        type: "teleop",
        seq: seq.current,
        focused: document.hasFocus(),
        visible: document.visibilityState === "visible",
        intent,
        mouse: { dx: m.dx, dy: m.dy, wheel: m.wheel },
        grip_toggle: m.toggles,
        arm_speed: armSpeed(),
        arm_joint: armJoint.current,
      });
      m.dx = m.dy = m.wheel = 0;
      m.toggles = 0;
    }, SEND_MS);
    return () => {
      window.removeEventListener("keydown", down, true);
      window.removeEventListener("keyup", up, true);
      window.removeEventListener("blur", release);
      document.removeEventListener("visibilitychange", release);
      window.clearInterval(timer);
    };
  }, [on]);

  // Mouse on the camera view (2026-11-15): click once to grab the mouse (pointer lock, Esc
  // releases) or hold the left button. Mouse up/down = gripper up/down, wheel = forward/back.
  // Both scaled by the operator's sensitivity.
  useEffect(() => {
    const el = area.current;
    if (!el || !on) return;
    const locked = () => document.pointerLockElement === el;
    const pdown = (e: PointerEvent) => {
      if (e.button !== 0) return;
      if (!locked()) {
        try { el.requestPointerLock(); } catch { /* falls back to hold-to-drag */ }
      }
      el.setPointerCapture(e.pointerId);
      mouse.current.drag = true;
      el.focus();
    };
    const pmove = (e: PointerEvent) => {
      if (!mouse.current.drag && !locked()) return;
      if (e.movementX || e.movementY) mouse.current.last = performance.now();
      mouse.current.dx += e.movementX * sensitivity();
      mouse.current.dy += e.movementY * sensitivity();
    };
    const pup = () => {
      mouse.current.drag = false;
    };
    const lockChange = () => {
      mouse.current.drag = false;
      setGrabbed(locked());
    };
    document.addEventListener("pointerlockchange", lockChange);
    const wheel = (e: WheelEvent) => {
      e.preventDefault();
      mouse.current.last = performance.now();
      mouse.current.wheel += Math.max(-3, Math.min(3, -e.deltaY / 100)) * sensitivity();
    };
    const ctx = (e: MouseEvent) => e.preventDefault();
    el.addEventListener("pointerdown", pdown);
    el.addEventListener("pointermove", pmove);
    el.addEventListener("pointerup", pup);
    el.addEventListener("pointercancel", pup);
    el.addEventListener("lostpointercapture", pup);
    el.addEventListener("wheel", wheel, { passive: false });
    el.addEventListener("contextmenu", ctx);
    return () => {
      el.removeEventListener("pointerdown", pdown);
      el.removeEventListener("pointermove", pmove);
      el.removeEventListener("pointerup", pup);
      el.removeEventListener("pointercancel", pup);
      el.removeEventListener("lostpointercapture", pup);
      el.removeEventListener("wheel", wheel);
      el.removeEventListener("contextmenu", ctx);
      document.removeEventListener("pointerlockchange", lockChange);
      if (locked()) document.exitPointerLock();
    };
  }, [on]);

  return { shown, area, dragging: () => mouse.current.drag, grabbed, jointMode, joint };
}

function LiveCamera({ feed }: { feed: string }) {
  // multipart JPEG from the backend (~15 fps while teleop is on); remount if the stream ends
  const [n, setN] = useState(0);
  return <img key={n} src={`/stream/${feed}.mjpg?n=${n}`} alt="" draggable={false} onError={() => window.setTimeout(() => setN((x) => x + 1), 1000)} />;
}

const PHASE: Record<TeleopView["phase"], [string, string]> = {
  off: ["OFF", "muted"],
  ready: ["READY", "ok"],
  starting: ["STARTING", "info"],
  active: ["DRIVING", "info"],
  ending: ["STOPPING", "info"],
  latched: ["HOLD", "warn"],
};

function Key({ code, label, held, wide }: { code: string; label: string; held: string[]; wide?: boolean }) {
  return (
    <span className={`tkey ${held.includes(code) ? "on" : ""} ${wide ? "wide" : ""}`} title={code}>
      {label}
    </span>
  );
}

function KeyMap({ held, dragging }: { held: string[]; dragging: boolean }) {
  return (
    <div className="tkeys" aria-label="keys held">
      <div className="trow">
        <Key code="KeyQ" label="Q ⟲" held={held} />
        <Key code="KeyW" label="W ↑" held={held} />
        <Key code="KeyE" label="E ⟳" held={held} />
        <Key code="KeyR" label="R tip↑" held={held} />
        <span className="tkey-note">base · pitch</span>
      </div>
      <div className="trow">
        <Key code="KeyA" label="A ←" held={held} />
        <Key code="KeyS" label="S ↓" held={held} />
        <Key code="KeyD" label="D →" held={held} />
        <Key code="KeyF" label="F tip↓" held={held} />
        <Key code="KeyG" label="G grip" held={held} />
      </div>
      <div className="trow">
        <Key code="KeyZ" label="Z roll−" held={held} />
        <Key code="KeyX" label="X roll+" held={held} />
        <Key code="KeyO" label="O open" held={held} />
        <Key code="KeyC" label="C close" held={held} />
      </div>
      <div className="trow">
        <Key code="ShiftLeft" label="Shift fast" held={held.map((k) => k.replace("Right", "Left"))} wide />
        <span className="tkey wide stopkey">Space STOP</span>
        <span className={`tkey wide ${dragging ? "on" : ""}`}>drag arm</span>
        <Key code="KeyJ" label="J joint mode" held={held} wide />
      </div>
    </div>
  );
}

function Hud({ t }: { t: TeleopView }) {
  const bv = t.base_vel;
  const row = (k: string, v: string, cls = "") => (
    <div className={`hudrow ${cls}`}>
      <span className="muted">{k}</span>
      <span className="mono">{v}</span>
    </div>
  );
  return (
    <div className="hud">
      {row("base", bv ? `${(bv.vx * 100).toFixed(1)} / ${(bv.vy * 100).toFixed(1)} cm/s · ${bv.wz_deg.toFixed(1)}°/s` : "—")}
      {row("gripper tcp", t.tcp ? `reach ${(t.tcp.reach * 100).toFixed(1)} · z ${(t.tcp.height * 100).toFixed(1)} cm · pitch ${t.tcp.pitch_deg.toFixed(0)}°` : "—")}
      {row("tcp speed", t.tcp_speed != null ? `${(t.tcp_speed * 100).toFixed(1)} cm/s` : "—")}
      {row("grip / roll", t.command ? `${t.command.joint_grip_L?.toFixed(0)}° / ${t.command.joint_wrist?.toFixed(0)}°` : "—")}
      {row("limits", `base ${t.limits.base_mps.map((v) => v * 100).join("|")} cm/s · ${t.limits.base_dps.join("|")}°/s · tcp ${t.limits.tcp_mps.map((v) => v * 100).join("|")} cm/s`)}
      {row("input", `${t.frame_age_ms ?? "—"} ms ago · p95 gap ${t.frame_gap_p95_ms ?? "—"} ms · dead-man ${t.limits.deadman_ms} ms`, (t.frame_age_ms ?? 0) > 200 ? "warnline" : "")}
      {row("collision", `${t.collision_mode === "full" ? "self + objects + OctoMap" : t.collision_mode === "no_octomap" ? "self + objects (OctoMap IGNORED)" : "SELF ONLY (table, cup, OctoMap IGNORED)"} · ${t.check_ms ?? "—"} ms`, t.collision_mode === "self_only" || t.check_errors ? "badline" : "")}
      {t.blocked && row("blocked", `${t.blocked}`, "badline")}
      {t.escape && row("contact", "escaping an existing contact", "warnline")}
      {t.governed.length > 0 && row("tracking", `waiting for ${t.governed.join(", ")} to catch up`, "warnline")}
      {t.ik_short && row("reach", "out of reach or at a joint limit", "warnline")}
      {t.limited.length > 0 && row("limited", t.limited.join(", "))}
    </div>
  );
}

function Recorder({ t, allowed }: { t: TeleopView; allowed: Snapshot["allowed"] }) {
  const [instr, setInstr] = useState(() => {
    try {
      return localStorage.getItem("hp.teleop.instruction") || "";
    } catch {
      return "";
    }
  });
  const r = t.recording;
  const save = (v: string) => {
    setInstr(v);
    try {
      localStorage.setItem("hp.teleop.instruction", v);
    } catch {
      /* private window */
    }
  };
  return (
    <div className="trec">
      {r.active ? (
        <>
          <span className="recdot" aria-hidden /> <b>REC</b>
          <span className="mono">
            {r.seconds?.toFixed(0)} s · {r.samples} samples{r.dropped ? ` · ${r.dropped} dropped` : ""}
          </span>
          <span className="instr" title={r.instruction ?? ""}>“{r.instruction}”</span>
          <button onClick={() => act("record_stop", { outcome: "success" })}>Save: success</button>
          <button onClick={() => act("record_stop", { outcome: "failure" })}>Save: failure</button>
          <button onClick={() => act("record_stop", { outcome: "discard" })}>Discard</button>
        </>
      ) : (
        <>
          <input
            value={instr}
            onChange={(e) => save(e.target.value)}
            placeholder="Language instruction, e.g. “pick up the red cup and place it on the drip tray”"
            aria-label="language instruction for the recording"
          />
          <button disabled={!allowed.record_start?.allowed || !instr.trim()} title={allowed.record_start?.reason ?? ""}
            onClick={() => act("record_start", { instruction: instr })}>
            ● Record episode
          </button>
          {r.last_result && <span className="muted mono small" title={r.root}>{r.last_result}</span>}
        </>
      )}
    </div>
  );
}

export function TeleopPanel({ snap }: { snap: Snapshot }) {
  const t = snap.teleop;
  const root = useRef<HTMLDivElement | null>(null);
  const [big, setBig] = useState(() => { try { return localStorage.getItem("homeplus.teleop.big") === "1"; } catch { return false; } });
  const toggleBig = () => { setBig((b) => { try { localStorage.setItem("homeplus.teleop.big", b ? "0" : "1"); } catch { /* blocked */ } return !b; }); };
  const fullscreen = () => {
    const el = root.current;
    if (!el) return;
    if (document.fullscreenElement) void document.exitFullscreen();
    else void el.requestFullscreen?.();
  };
  const on = !!t?.mode && snap.control.you;
  const { shown, area, dragging, grabbed, jointMode, joint } = useTeleopInput(on);
  const [, force] = useState(0);
  useEffect(() => {
    if (!on) return;
    const id = window.setInterval(() => force((n) => n + 1), 100); // drag indicator
    return () => window.clearInterval(id);
  }, [on]);
  if (!t) return <div className="tele-empty">This console backend has no teleop.</div>;
  const a = snap.allowed;
  const [phase, cls] = PHASE[t.phase];
  const d405 = snap.feeds.find((f) => f.id === "d405_color");
  return (
    <div ref={root} className={`teleop ${t.mode ? "is-on" : ""} ${big ? "big" : ""}`}>
      <div className="tbar">
        <button className="tog" onClick={toggleBig} title="Larger camera, smaller side panel">{big ? "Normal view" : "Large view"}</button>
        <button className="tog" onClick={fullscreen} title="Fill the screen (Esc to leave)">⛶ Full screen</button>
        {t.mode ? (
          <button className="tog on" disabled={!a.teleop_off?.allowed} onClick={() => act("teleop_off")}>
            Teleop ON · turn off
          </button>
        ) : (
          <button className="tog" disabled={!a.teleop_on?.allowed} title={a.teleop_on?.reason ?? ""} onClick={() => act("teleop_on")}>
            Turn teleop on
          </button>
        )}
        <span className={`tphase ${cls}`}>{phase}</span>
        <span className="treason" title={t.reason ?? ""}>
          {!t.mode
            ? a.teleop_on?.allowed
              ? "keyboard + mouse control of base, arm and gripper"
              : `can't start: ${a.teleop_on?.reason}`
            : t.phase === "latched"
              ? `${t.reason} — release all keys and buttons to continue`
              : t.phase === "ready"
                ? "press a key or drag on the camera to move"
                : t.blocked
                  ? `blocked: ${t.blocked}`
                  : ""}
        </span>
        {t.mode && (
          <label className={`tcoll ${t.collision_mode === "self_only" ? "badline" : ""}`}
            title="What blocks motion. Full: self-collision, scene objects (table, cup) and OctoMap voxels. Use the others only when the scene is wrong, e.g. voxels or the cup box around the cup you are grasping.">
            collision
            <select value={t.collision_mode ?? "no_octomap"} disabled={!a.teleop_collision?.allowed}
              onChange={(e) => act("teleop_collision", { mode: e.target.value })}>
              <option value="full">self + objects + OctoMap</option>
              <option value="no_octomap">self + objects (no OctoMap)</option>
              <option value="self_only">self-collision only</option>
            </select>
          </label>
        )}
        {t.mode && !snap.control.you && <span className="badline">teleop belongs to {t.owner_label}; you are view-only</span>}
      </div>
      <div className="tmain">
        <div ref={area} className={`tcam ${on ? "live" : ""} ${dragging() ? "dragging" : ""}`} tabIndex={0}
          aria-label="D405 hand camera; hold the left mouse button and move to drive the gripper">
          {d405 && d405.status !== "off" ? <LiveCamera feed="d405_color" /> : <div className="veil off"><b>NO FRAMES</b></div>}
          <span className="tag">D405 hand · {d405?.status ?? "—"}</span>
          {on && (
            <div className="tcam-help">
              <b>{jointMode ? `JOINT MODE: ${JOINTS[joint][2]}` : "GRIPPER MODE"}</b>{" · "}
              {jointMode
                ? "mouse ↑↓ or wheel turns the joint · 1 shoulder 2 elbow 3 pitch 4 roll · J = gripper mode"
                : "mouse ↑↓ = up/down · wheel = forward/back · R/F tilt · J = joint mode"}
              {grabbed ? " · Esc releases the mouse" : " · click to grab the mouse"}
            </div>
          )}
        </div>
        <div className="tside">
          <div className="t3d">
            <Plan3D plan={null} planAge={null} now={snap.joints.si} scene={snap.scene ?? null} />
          </div>
          <SensitivitySlider />
          <KeyMap held={shown} dragging={dragging()} />
          <Hud t={t} />
        </div>
      </div>
      {t.mode && <Recorder t={t} allowed={a} />}
    </div>
  );
}
