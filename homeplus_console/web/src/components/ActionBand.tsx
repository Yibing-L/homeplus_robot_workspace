import { Home, Move, RotateCcw, ShieldAlert, Square, Undo2, X } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { act, useConsole } from "../store";
import type { Approval, Flow, NextAction, Snapshot } from "../types";
import { fmtAge, fmtClock, JOINT_LABEL, signed, useSinceSnapshot } from "../util";
import { type Arm, ArmConfirm, ArmDetails, ManualMotion } from "./Recovery";
import { TopDown } from "./TopDown";

function Mark({ sev }: { sev: string }) {
  return <span className={`mark ${sev}`} aria-hidden style={{ marginRight: 10, verticalAlign: "middle" }} />;
}

function ResultLine({ name }: { name: string }) {
  const snap = useConsole((s) => s.snap);
  const local = useConsole((s) => s.lastResult[name]);
  const tracked = snap ? [...snap.tracked_actions].reverse().find((t) => t.action === name) : undefined;
  const useTracked = tracked && (!local || tracked.t_sent * 1000 >= local.at - 2000);
  const r = useTracked ? tracked : local;
  const at = useTracked ? tracked!.t_sent * 1000 : local?.at ?? 0;
  if (!r || Date.now() - at > 10000) return null; // a result is news, not state
  const started = useTracked && r.status === "confirmed" && /running$/.test(r.detail);
  const cls = started ? "running" : !useTracked && r.status === "confirmed" ? "noted" : r.status;
  return <div className={`result-line ${cls}`}>{r.detail}</div>;
}

function Bay({ motion = true }: { motion?: boolean }) {
  return <div className={`bay-label ${motion ? "motion" : ""}`}>{motion ? "Causes motion" : "Controls"}</div>;
}

// ------------------------------------------------------------------ approval (§4.4)
function HoldToApprove({ approval, allowed, reason }: { approval: Approval; allowed: boolean; reason: string | null }) {
  const [holding, setHolding] = useState(false);
  const timer = useRef<number>();
  const start = () => {
    if (!allowed) return;
    setHolding(true);
    timer.current = window.setTimeout(() => {
      setHolding(false);
      void act("approve", { plan_id: approval.plan_id });
    }, 1000);
  };
  const cancel = () => {
    window.clearTimeout(timer.current);
    setHolding(false);
  };
  return (
    <button
      className={`btn motion hold ${holding ? "holding" : ""}`}
      disabled={!allowed}
      onPointerDown={start}
      onPointerUp={cancel}
      onPointerLeave={cancel}
      onKeyDown={(e) => (e.key === " " || e.key === "Enter") && !e.repeat && (e.preventDefault(), start())}
      onKeyUp={(e) => (e.key === " " || e.key === "Enter") && cancel()}
      aria-label={`Hold for one second to approve plan ${approval.plan_id}`}
      title={reason ?? ""}
    >
      <span className="fill" />
      <span>
        <Move size={20} aria-hidden /> {approval.preview_ready ? `Hold to approve${approval.plan_id != null ? ` #${approval.plan_id}` : ""}` : "Waiting for plan preview…"}
      </span>
    </button>
  );
}

function ApprovalBand({ snap, approval }: { snap: Snapshot; approval: Approval }) {
  const { since, lost } = useSinceSnapshot();
  const remaining = approval.remaining_sec == null ? null : Math.max(0, approval.remaining_sec - (approval.state_age ?? 0) - since);
  const frac = remaining == null ? 0 : remaining / approval.timeout_sec;
  const low = remaining != null && remaining < 15;
  const p = approval.preview;
  const a = snap.allowed;
  const seq = snap.executor.sequence;
  const baseMoves = p ? Math.hypot(p.base_delta.dx, p.base_delta.dy) > 0.02 || Math.abs(p.base_delta.dtheta_deg) > 2 : false;
  useEffect(() => {
    // one short tone when an approval appears (§4.4)
    try {
      const ctx = new AudioContext();
      const o = ctx.createOscillator();
      const g = ctx.createGain();
      o.frequency.value = 880;
      g.gain.setValueAtTime(0.08, ctx.currentTime);
      g.gain.exponentialRampToValueAtTime(0.0001, ctx.currentTime + 0.35);
      o.connect(g).connect(ctx.destination);
      o.start();
      o.stop(ctx.currentTime + 0.36);
    } catch {
      /* audio blocked until the page has been interacted with */
    }
  }, [approval.plan_id]);
  return (
    <div className="band warn" role="region" aria-label="Approval">
      <div>
        <h3 className="mono">{approval.step}</h3>
        <div className="meta">
          {p?.undo ? "undo · " : ""}
          {approval.plan_id != null ? `plan #${approval.plan_id}` : "unscoped approval (Bool topic, §7.3 pending)"}
          {seq ? ` · step ${seq.step_index}/${seq.step_count}` : ""}
        </div>
        <div className={`count ${low ? "low" : ""}`} aria-live="off">
          {lost || remaining == null ? "?" : Math.ceil(remaining)}
          <small> s left</small>
        </div>
        <div className={`drain ${low ? "low" : ""}`}>
          <i style={{ width: `${lost ? 0 : frac * 100}%` }} />
        </div>
        <div className="why-line" style={{ marginTop: 6 }}>
          {lost ? "Link lost: countdown unknown." : `Executor deadline ${fmtClock(approval.deadline_wall, true)} · rejects on timeout`}
        </div>
      </div>
      <div>
        <div className="label" style={{ marginBottom: 6 }}>
          What will move{p?.command === "return_to_start" ? " · return to start" : ""}
        </div>
        {p ? (
          <div className={baseMoves ? "plan-split" : ""}>
            <div>
              {p.undo && (
                <div className={`method ${p.undo.method === "fallback_plan" ? "fallback" : ""}`} style={{ marginBottom: 8 }}>
                  {p.undo.method === "reverse_replay"
                    ? `Reverse replay of ${p.undo.traj_file ?? p.undo.of_step}`
                    : p.undo.method === "restore_gripper"
                      ? `Restore gripper: ${p.undo.reason}`
                      : `Fallback: ${p.undo.reason}`}
                </div>
              )}
              <table className="move-table">
                <tbody>
                  <tr>
                    <td>Base</td>
                    <td className="mono">
                      Δx {signed(p.base_delta.dx, 2, " m")} · Δy {signed(p.base_delta.dy, 2, " m")} · Δθ {signed(p.base_delta.dtheta_deg, 1, "°")}
                    </td>
                  </tr>
                  {p.joints.map((j) => (
                    <tr key={j.name}>
                      <td>{JOINT_LABEL[j.name] ?? j.name}</td>
                      <td className="mono">
                        {j.start.toFixed(0)}° → {j.end.toFixed(0)}°
                        <span className="jbar" aria-hidden>
                          <s />
                          <i style={{ left: `${50 + Math.min(j.start, j.end) / 1.8}%`, width: `${Math.max(1, Math.abs(j.end - j.start) / 1.8)}%` }} />
                        </span>
                      </td>
                    </tr>
                  ))}
                  <tr>
                    <td>Motion</td>
                    <td className="mono">
                      {p.duration_sec.toFixed(1)} s · {p.waypoints} wp · TCP end ({p.tcp_end.map((v) => v.toFixed(2)).join(", ")})
                    </td>
                  </tr>
                  {!p.undo && (
                    <tr>
                      <td>Target</td>
                      <td className="mono">
                        {p.target_distance_m != null
                          ? `${(p.target_distance_m * 100).toFixed(1)} cm from TCP end · detection ${fmtAge(p.target_age_sec)} old`
                          : "no detected target for this step"}
                      </td>
                    </tr>
                  )}
                </tbody>
              </table>
            </div>
            {baseMoves && <TopDown now={p.base_start} target={p.base_end} label="base path, top-down" />}
          </div>
        ) : (
          <div className="empty" style={{ background: "var(--hatch)", padding: 14 }}>
            Plan preview for #{approval.plan_id} has not arrived. Approve stays disabled until it does.
          </div>
        )}
      </div>
      <div>
        <Bay />
        <HoldToApprove approval={approval} allowed={!!a.approve?.allowed} reason={a.approve?.reason ?? null} />
        <div style={{ display: "flex", gap: 8 }}>
          <button className="btn" style={{ flex: 1 }} disabled={!a.reject?.allowed} onClick={() => act("reject")}>
            <X size={15} aria-hidden /> Reject
          </button>
          <button className="btn" style={{ flex: 1 }} disabled={!a.cancel?.allowed} onClick={() => act("cancel")}>
            <Square size={13} aria-hidden /> Cancel sequence
          </button>
        </div>
        {!a.approve?.allowed && a.approve?.reason && <div className="why-line">{a.approve.reason}</div>}
        <ResultLine name="approve" />
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ set start here (§5.3)
function FlowBand({ snap, flow }: { snap: Snapshot; flow: Flow }) {
  const a = snap.allowed;
  const sev = flow.status === "error" ? "bad" : flow.status === "done" ? "ok" : "info";
  const cur = flow.steps[flow.step_index];
  return (
    <div className={`band ${sev}`} role="region" aria-label="Set start here">
      <div>
        <h3>Set start here</h3>
        <div className="meta">{flow.method === "zero" ? "ZERO command, no reset" : "Arduino reset (arm re-homes)"}</div>
        <ol className="flowsteps">
          {flow.steps.map((s, i) => (
            <li key={s.id} className={s.status}>
              <Mark sev={s.status === "done" ? "ok" : s.status === "error" ? "bad" : s.status === "active" ? "info" : "unknown"} />
              {i + 1}. {s.title}
            </li>
          ))}
        </ol>
      </div>
      <div>
        <h3>{flow.status === "done" ? "Start set at the robot's current spot" : cur.title}</h3>
        <div className="meta">
          step {flow.step_index + 1} of {flow.steps.length}
          {flow.status === "active" ? ` · ${fmtAge(flow.step_age)}` : ""} · {flow.reason}
        </div>
        <div className="fix">{flow.status === "error" ? flow.error : flow.waiting_for}</div>
        {snap.estop_declaration && flow.status !== "done" && (
          <div className="declared-note" style={{ marginTop: 8 }}>
            E-stop declared pressed by {snap.estop_declaration.by} at {fmtClock(snap.estop_declaration.at, true)}. The console can't
            see the E-stop: this is your declaration.
          </div>
        )}
        {flow.crosscheck === "contradicted" && (
          <div className="contradicted" style={{ marginTop: 8 }}>
            Motors are powered. The E-stop was NOT pressed. Recorded in the audit log.
          </div>
        )}
        {flow.crosscheck === "consistent" && flow.status !== "done" && (
          <div className="consistent" style={{ marginTop: 8 }}>
            Arm stayed folded after the reset: consistent with the E-stop pressed.
          </div>
        )}
      </div>
      <div>
        <Bay motion={flow.method === "reset" && cur.id === "zero"} />
        {flow.options.map((o) => (
          <button
            key={o.choice}
            className={`btn ${o.primary ? "primary lg" : ""}`}
            disabled={!a.flow_confirm?.allowed}
            onClick={() => act("flow_confirm", { choice: o.choice })}
          >
            {o.label}
          </button>
        ))}
        {flow.status === "error" && (
          <button className="btn primary lg" disabled={!a.flow_retry?.allowed} onClick={() => act("flow_retry")}>
            <RotateCcw size={18} aria-hidden /> Retry this step
          </button>
        )}
        {flow.status === "done" ? (
          <div className="consistent">Origin confirmed {fmtClock(snap.origin.at, true)} by {snap.origin.by}.</div>
        ) : (
          <button className="btn" disabled={!a.flow_cancel?.allowed} onClick={() => act("flow_cancel")}>
            Cancel
          </button>
        )}
        {flow.options.length === 0 && flow.status === "active" && <div className="why-line">Advances on its own when the console observes the result.</div>}
        <ResultLine name="flow_confirm" />
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ abort → recover (§5.4)
function RecoverBand({ snap, top, rest, arm, setArm }: BandProps & { top: NextAction; rest: NextAction[] }) {
  const a = snap.allowed;
  const [inspected, setInspected] = useState(false);
  const lr = snap.executor.last_result;
  const rec = snap.recovery;
  const res = snap.executor.resumable;
  const gate = (name: string) => inspected && !!a[name]?.allowed;
  const pick = (kind: Arm["kind"], args: Record<string, unknown> = {}) => setArm({ kind, args: { ...args, inspected: true } });
  return (
    <div className="band bad" role="region" aria-label="Recover">
      <div>
        <h3>
          <Mark sev="bad" />
          {lr?.outcome === "cancelled" ? "Cancelled" : "Aborted"} at <span className="mono">{lr?.step}</span>
        </h3>
        <div className="meta">{fmtClock(lr?.t, true)} · {lr?.command}</div>
        <div className="problem" style={{ marginTop: 8 }}>
          {lr?.reason}
        </div>
      </div>
      <div>
        {arm ? (
          <ArmDetails snap={snap} arm={arm} setArm={setArm} />
        ) : (
          <>
            <div className="fix">{top.fix}</div>
            <div className="others">
              {rec && (
                <div className="mono">
                  {rec.home_distance_m.toFixed(2)} m from start · {rec.undo_count} undoable step{rec.undo_count === 1 ? "" : "s"}
                  {snap.executor.holding_cup ? " · holding the cup" : ""}
                </div>
              )}
              {rest.slice(0, 2).map((n) => (
                <div key={n.id}>
                  <Mark sev={n.severity} />
                  <span>
                    <b>{n.problem}</b>
                    {n.fix ? ` · ${n.fix}` : ""}
                  </span>
                </div>
              ))}
              <div>
                <button className="btn sm" disabled={!inspected || !a.ack_fault?.allowed} onClick={() => act("ack_fault").then(() => setInspected(false))}>
                  <ShieldAlert size={14} aria-hidden /> Acknowledge without recovering
                </button>
              </div>
            </div>
          </>
        )}
      </div>
      <div className={arm ? "" : "recover"}>
        <Bay />
        {arm ? (
          <ArmConfirm snap={snap} arm={arm} setArm={setArm} />
        ) : (
          <>
            <label className="ack">
              <input type="checkbox" checked={inspected} onChange={(e) => setInspected(e.target.checked)} />
              I've inspected the robot
            </label>
            <button className="btn motion" disabled={!gate("undo_sequence")} title={a.undo_sequence?.reason ?? ""} onClick={() => pick("undo_sequence")}>
              <Undo2 size={15} aria-hidden /> Undo {rec?.undo_count ?? 0} steps
            </button>
            <button className="btn motion" disabled={!gate("return_to_start")} title={a.return_to_start?.reason ?? ""} onClick={() => pick("return", { release_cup: true })}>
              <Home size={15} aria-hidden /> Return to start
            </button>
            {snap.executor.capabilities.resume && res && (
              <button className="btn motion" disabled={!gate("resume")} title={a.resume?.reason ?? ""} onClick={() => pick("resume", { step_index: res.step_index })}>
                <RotateCcw size={15} aria-hidden /> Retry from step {res.step_index}
              </button>
            )}
            {!inspected && <div className="why-line">Recovery moves the robot: look at it first.</div>}
          </>
        )}
        <ResultLine name="undo_sequence" />
        <ResultLine name="return_to_start" />
        <ResultLine name="resume" />
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ problem + its one fix
function ProblemBand({ snap, top, rest, arm, setArm }: BandProps & { top: NextAction; rest: NextAction[] }) {
  const a = snap.allowed;
  const [inspected, setInspected] = useState(false);
  const busy = snap.executor.busy;
  const isAck = top.action?.name === "ack_fault";
  return (
    <div className={`band ${top.severity}`} role="region" aria-label="Next action">
      <div>
        <h3>
          <Mark sev={top.severity} />
          {top.problem}
        </h3>
        <div className="meta">
          {snap.robot.title} · since {fmtClock(snap.robot.since_wall, true)}
        </div>
        <div className="problem" style={{ marginTop: 8 }}>
          {top.detail}
        </div>
      </div>
      <div>
        {arm ? (
          <ArmDetails snap={snap} arm={arm} setArm={setArm} />
        ) : (
          <>
            <div className="fix">{top.fix}</div>
            {rest.length > 0 && (
              <div className="others">
                {rest.slice(0, 3).map((n) => (
                  <div key={n.id}>
                    <Mark sev={n.severity} />
                    <span>
                      <b>{n.problem}</b>
                      {n.fix ? ` · ${n.fix}` : ""}
                    </span>
                  </div>
                ))}
                {rest.length > 3 && <span className="muted">+{rest.length - 3} more</span>}
              </div>
            )}
            <div className="others">
              {top.flow === "set_start" && (
                <div>
                  <button
                    className="btn primary"
                    disabled={!a.flow_set_start?.allowed}
                    title={a.flow_set_start?.reason ?? ""}
                    onClick={() => act("flow_set_start", { reason: top.problem })}
                  >
                    <RotateCcw size={16} aria-hidden /> Set start here…
                  </button>
                </div>
              )}
              {top.action && isAck && (
                <div>
                  <label className="ack">
                    <input type="checkbox" checked={inspected} onChange={(e) => setInspected(e.target.checked)} />
                    I've inspected the robot
                  </label>
                  <button className="btn primary" disabled={!inspected || !a.ack_fault?.allowed} onClick={() => act("ack_fault").then(() => setInspected(false))}>
                    <ShieldAlert size={16} aria-hidden /> Acknowledge
                  </button>
                </div>
              )}
              {top.action && !isAck && (
                <div>
                  <button
                    className="btn primary"
                    disabled={!a[top.action.name]?.allowed}
                    title={a[top.action.name]?.reason ?? ""}
                    onClick={() => act(top.action!.name, top.action!.args ?? {})}
                  >
                    {top.action.label}
                  </button>
                </div>
              )}
              {!top.flow && !top.action && <div className="why-line">No console action fixes this: it needs hands on the robot or the Mac.</div>}
              {top.action && <ResultLine name={top.action.name} />}
              <ResultLine name="flow_set_start" />
            </div>
          </>
        )}
      </div>
      <div>
        <Bay motion={!busy} />
        {arm ? (
          <ArmConfirm snap={snap} arm={arm} setArm={setArm} />
        ) : busy ? (
          <button className="btn lg" disabled={!a.cancel?.allowed} onClick={() => act("cancel")}>
            <Square size={13} aria-hidden /> Cancel sequence
          </button>
        ) : (
          <ManualMotion snap={snap} setArm={setArm} />
        )}
        <ResultLine name="return_to_start" />
        <ResultLine name="undo_step" />
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ running
function RunningBand({ snap }: { snap: Snapshot }) {
  const seq = snap.executor.sequence;
  const a = snap.allowed;
  const warn = snap.next_actions[0];
  return (
    <div className="band info" role="region" aria-label="Running">
      <div>
        <h3 className="mono">{seq?.step_name ?? "—"}</h3>
        <div className="meta">
          {snap.executor.command} · step {seq?.step_index}/{seq?.step_count} · {seq?.phase}
        </div>
      </div>
      <div>
        <div className="fix">
          {seq?.phase === "detecting"
            ? "Waiting for a stable detection of the target."
            : seq?.phase === "planning"
              ? "Planning the next motion."
              : seq?.phase === "verifying"
                ? "Checking the measured endpoint."
                : "The robot is moving. Keep clear; STOP is top-left (Esc)."}
        </div>
        {warn && (
          <div className="others">
            <div>
              <Mark sev={warn.severity} />
              <span>
                <b>{warn.problem}</b> · {warn.fix}
              </span>
            </div>
          </div>
        )}
      </div>
      <div>
        <Bay motion={false} />
        <button className="btn lg" disabled={!a.cancel?.allowed} onClick={() => act("cancel")}>
          <Square size={15} aria-hidden /> Cancel sequence
        </button>
        <ResultLine name="start_task" />
        <ResultLine name="return_to_start" />
        <ResultLine name="cancel" />
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ idle: preflight + start (§5.2)
function StartBand({ snap, arm, setArm }: BandProps) {
  const a = snap.allowed;
  const [task, setTask] = useState(1);
  const fails = snap.preflight.filter((p) => !p.ok);
  const decl = snap.declared;
  const { lost } = useSinceSnapshot();
  const ready = !lost && snap.robot.state === "IDLE_READY" && a.start_task?.allowed;
  return (
    <div className={`band ${ready ? "ok" : "warn"}`} role="region" aria-label="Preflight and start">
      <div>
        <h3>{task === 1 ? "Pick up the cup" : "Press the square button"}</h3>
        <div className="meta">
          {lost ? "preflight unknown: console link lost" : ready ? "preflight passed" : `not ready · ${fails.length + decl.filter((d) => !d.ok).length} blockers`}
        </div>
        <div style={{ display: "flex", gap: 6, marginTop: 12 }} role="radiogroup" aria-label="Task">
          {[
            [1, "Task 1 · pickup"],
            [2, "Task 2 · button"],
          ].map(([id, label]) => (
            <button key={id} className={`btn sm ${task === id ? "primary" : ""}`} role="radio" aria-checked={task === id} onClick={() => setTask(id as number)}>
              {label}
            </button>
          ))}
        </div>
      </div>
      <div>
        {arm ? (
          <ArmDetails snap={snap} arm={arm} setArm={setArm} />
        ) : (
          <div className={`checks ${lost ? "unknown" : ""}`}>
            {snap.preflight.map((p) => (
              <div key={p.id} className={p.ok ? "" : "ko"} title={p.detail ?? ""}>
                <Mark sev={lost ? "unknown" : p.ok ? "ok" : "bad"} />
                <span>
                  {p.label}
                  {!p.ok && p.detail ? ` · ${p.detail}` : ""}
                </span>
              </div>
            ))}
            {decl.map((d) => (
              <label key={d.id} title={d.auto ? "Origin confirmed in the last 10 min" : "Operator declaration, recorded in the audit log"}>
                <input type="checkbox" checked={d.ok} disabled={d.auto || !a.declare?.allowed} onChange={(e) => act("declare", { item: d.id, value: e.target.checked })} />
                <span>
                  {d.label}
                  <span className="muted"> · {d.auto ? "auto" : d.declared ? `declared ${fmtClock(d.declared.at)}` : "declare"}</span>
                </span>
              </label>
            ))}
          </div>
        )}
      </div>
      <div>
        <Bay />
        {arm ? (
          <ArmConfirm snap={snap} arm={arm} setArm={setArm} />
        ) : (
          <>
            <button className="btn motion lg" disabled={!a.start_task?.allowed} onClick={() => setArm({ kind: "start", args: { task } })}>
              <Move size={20} aria-hidden /> Start {task === 1 ? "pickup" : "button press"}…
            </button>
            {!a.start_task?.allowed && <div className="why-line">{a.start_task?.reason}</div>}
            <ManualMotion snap={snap} setArm={setArm} />
          </>
        )}
        <ResultLine name="start_task" />
        <ResultLine name="return_to_start" />
        <ResultLine name="undo_step" />
      </div>
    </div>
  );
}

interface BandProps {
  snap: Snapshot;
  arm: Arm | null;
  setArm: (a: Arm | null) => void;
}

export function ActionBand({ snap }: { snap: Snapshot }) {
  const [arm, setArm] = useState<Arm | null>(null);
  const flow = snap.flow;
  const mode = flow && flow.status !== "cancelled" && (flow.status !== "done" || flow.step_age < 8)
    ? "flow"
    : snap.approval
      ? "approval"
      : "other";
  // an armed confirmation never survives a change of situation
  const key = `${mode}:${snap.robot.state}`;
  const lastKey = useRef(key);
  useEffect(() => {
    if (lastKey.current !== key) {
      lastKey.current = key;
      setArm(null);
    }
  }, [key]);
  if (mode === "flow") return <FlowBand snap={snap} flow={flow!} />;
  if (mode === "approval") return <ApprovalBand snap={snap} approval={snap.approval!} />;
  const bad = snap.next_actions.filter((n) => n.severity === "bad");
  if (snap.executor.busy && bad.length === 0) return <RunningBand snap={snap} />;
  const abort = bad.find((n) => n.id === "abort");
  if (abort && !snap.executor.busy) return <RecoverBand snap={snap} top={abort} rest={snap.next_actions.filter((n) => n !== abort)} arm={arm} setArm={setArm} />;
  if (bad.length > 0) return <ProblemBand snap={snap} top={bad[0]} rest={snap.next_actions.filter((n) => n !== bad[0])} arm={arm} setArm={setArm} />;
  return <StartBand snap={snap} arm={arm} setArm={setArm} />;
}
