import { Undo2 } from "lucide-react";
import { useState } from "react";
import type { LogLine, Snapshot, Step } from "../types";
import { fmtClock } from "../util";

const MOTION_TYPES = new Set(["pose", "joint_state", "gripper"]);

function stepClass(s: Step, awaiting: boolean): string {
  if (awaiting) return "awaiting";
  return s.status;
}

function StepRow({ s, i, awaiting, logs }: { s: Step; i: number; awaiting: boolean; logs: LogLine[] }) {
  const [open, setOpen] = useState(false);
  const dur = s.t_start && s.t_end ? `${(s.t_end - s.t_start).toFixed(1)} s` : "";
  const auto = MOTION_TYPES.has(s.type) && !s.requires_approval;
  const lines = s.t_start
    ? logs.filter((l) => l.important && l.t_wall >= s.t_start! - 0.05 && l.t_wall <= (s.t_end ?? Infinity) + 0.05 &&
        ["gesture_motion_executor", "arduino_bridge"].includes(l.node))
    : [];
  let status: string;
  if (awaiting) status = "approve";
  else if (s.status === "done") status = s.note || dur || "done";
  else if (s.status === "running") status = "running";
  else if (s.status === "failed") status = "failed";
  else if (s.status === "cancelled") status = "cancelled";
  else if (s.status === "skipped") status = "skipped";
  else status = "";
  return (
    <li className={`step ${stepClass(s, awaiting)}`}>
      <button onClick={() => setOpen((v) => !v)} aria-expanded={open}>
        <span className="n">{i + 1}</span>
        <span className="nm">{s.name}</span>
        <span className="st">
          {auto && <span className="auto" title="Moves the robot without approval (require_trajectory_approval: false)">auto</span>}
          {s.requires_approval && !awaiting && s.status === "pending" && <span className="auto">approval</span>}
          {status}
        </span>
      </button>
      {open && (
        <div className="step-logs">
          {s.status === "failed" && s.note && <div className="ERROR">{s.note}</div>}
          {lines.length === 0 && <div className="muted">{s.t_start ? "No matching log lines." : "Not started."}</div>}
          {lines.slice(-8).map((l) => (
            <div key={l.seq} className={l.level}>
              {fmtClock(l.t_wall, true)} {l.text}
            </div>
          ))}
        </div>
      )}
    </li>
  );
}

export function Timeline({ snap }: { snap: Snapshot }) {
  const ex = snap.executor;
  const steps = ex.steps;
  const awaitingStep = snap.approval?.step;
  const lr = ex.last_result;
  const name = ex.sequence?.name ?? ex.command ?? lr?.command;
  const started = steps[0]?.t_start;
  return (
    <section className="panel" aria-label="Sequence timeline">
      <div className="panel-head">
        <h2>
          Sequence{name ? " · " : ""}
          {name && <span className="mono">{name}</span>}
        </h2>
        <span className="label">{!ex.fresh ? "executor unknown" : started ? `started ${fmtClock(started, true)}` : "idle"}</span>
      </div>
      <div className="panel-body">
        {!ex.fresh && (
          <div className="empty" style={{ background: "var(--hatch)" }}>
            <b style={{ color: "var(--ink)" }}>Executor not reporting</b>
            <span>Step states below are the last ones seen; the true state is unknown.</span>
          </div>
        )}
        {steps.length === 0 ? (
          <div className="empty">
            <b>No sequence yet this session</b>
          </div>
        ) : (
          <ol className="steps">
            {steps.map((s, i) => (
              <StepRow key={s.name + i} s={s} i={i} awaiting={s.name === awaitingStep && s.status === "running"} logs={snap.logs} />
            ))}
          </ol>
        )}
        {lr && !ex.busy && lr.outcome !== "succeeded" && (
          <div className="result-card bad">
            <b>
              {lr.outcome === "cancelled" ? "Cancelled" : "Aborted"} at {lr.step ?? "—"} · {fmtClock(lr.t, true)}
            </b>
            <span>{lr.reason}</span>
          </div>
        )}
        {lr && !ex.busy && lr.outcome === "succeeded" && (
          <div className="result-card ok">
            <b>{lr.command} complete · {fmtClock(lr.t, true)}</b>
          </div>
        )}
        {ex.undo_stack.length > 0 && (
          <>
            <div className="undo-head">
              <span className="label">Undo stack · newest first</span>
              <span className="muted" style={{ fontSize: 12 }}>
                {ex.undo_stack.filter((r) => r.reversible).length} reversible
              </span>
            </div>
            <ol className="steps">
              {ex.undo_stack.map((r) => (
                <li key={r.undo_id} className={`step undo ${r.reversible ? "" : "skipped"}`}>
                  <button title={r.traj_file ?? undefined}>
                    <span className="n"><Undo2 size={13} aria-hidden /></span>
                    <span className="nm">{r.step}</span>
                    <span className="st">
                      {!r.reversible ? "nothing to undo" : r.type === "gripper" ? `gripper → ${r.gripper_before.toFixed(0)}°` : "reverse replay"}
                    </span>
                  </button>
                </li>
              ))}
            </ol>
          </>
        )}
        <DetectionHistory snap={snap} />
      </div>
    </section>
  );
}

// Detection history (§4.2): the last poses the start gate is judged on, newest first.
function DetectionHistory({ snap }: { snap: Snapshot }) {
  const w = snap.perception.window;
  const [lo, hi] = w.z_band;
  const rows = [...w.samples].reverse().slice(0, 8);
  const st = snap.perception.state;
  const untrusted = ["CAMERA_DOWN", "CAMERA_FROZEN", "DETECTOR_OLD_FRAMES", "DETECTOR_DOWN"].includes(st);
  return (
    <div className={`dethist ${untrusted ? "untrusted" : ""}`}>
      {untrusted && <div className="untrusted-note">{snap.perception.title}: these poses come from old frames</div>}
      <div className="undo-head">
        <span className="label">Detections · task {w.task_id}</span>
        <span className="muted" style={{ fontSize: 12 }}>
          {w.consecutive} consecutive · spread {w.spread_cm ?? "—"} cm · z band {lo.toFixed(2)}–{hi.toFixed(2)} m
        </span>
      </div>
      {rows.length === 0 ? (
        <div className="empty">No poses in the last 20 s.</div>
      ) : (
        <table className="det-table">
          <thead>
            <tr>
              <th>age</th>
              <th>x</th>
              <th>y</th>
              <th>z</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((r, i) => (
              <tr key={i} className={r.z < lo || r.z > hi ? "out" : ""}>
                <td>{r.age.toFixed(1)} s</td>
                <td>{r.x.toFixed(3)}</td>
                <td>{r.y.toFixed(3)}</td>
                <td>{r.z.toFixed(3)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}
