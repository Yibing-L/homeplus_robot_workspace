import { Hand, Home, Move, RotateCcw, Undo2 } from "lucide-react";
import { act } from "../store";
import type { Snapshot } from "../types";
import { TopDown } from "./TopDown";

// Motion is confirmed inside the motion bay, never in a modal: the feeds and health stay visible
// at the moment the operator decides to move the robot, and STOP is never covered.

export type ArmKind = "start" | "return" | "undo_step" | "undo_sequence" | "gripper_open" | "gripper_close" | "resume";
export interface Arm {
  kind: ArmKind;
  args: Record<string, unknown>;
}

const ACTION: Record<ArmKind, string> = {
  start: "start_task",
  return: "return_to_start",
  undo_step: "undo_step",
  undo_sequence: "undo_sequence",
  gripper_open: "gripper_open",
  gripper_close: "gripper_close",
  resume: "resume",
};

function confirmLabel(arm: Arm, snap: Snapshot): string {
  switch (arm.kind) {
    case "start":
      return arm.args.task === 2 ? "Start button press" : "Start pickup";
    case "return":
      return "Plan return to start";
    case "undo_step":
      return "Plan undo";
    case "undo_sequence":
      return `Plan undo of ${snap.recovery?.undo_count ?? 0} steps`;
    case "gripper_open":
      return "Open gripper";
    case "gripper_close":
      return "Close gripper";
    case "resume":
      return `Retry from step ${snap.executor.resumable?.step_index ?? "?"}`;
  }
}

export function ArmDetails({ snap, arm, setArm }: { snap: Snapshot; arm: Arm; setArm: (a: Arm | null) => void }) {
  const r = snap.recovery;
  const ex = snap.executor;
  if (arm.kind === "start") {
    const w = snap.perception.window;
    const baseX = snap.joints.values?.base_x ?? 0;
    const drive = w.position ? Math.max(0, w.position[0] - baseX - 0.53) : null;
    return (
      <>
        <h3>Start {arm.args.task === 2 ? "press_square_button" : "pick_up_coffee"}?</h3>
        <div className="meta mono">
          target ({w.position?.map((v) => v.toFixed(2)).join(", ") ?? "—"}) m · {w.consecutive} consecutive · spread {w.spread_cm ?? "—"} cm
        </div>
        <div className="moves">
          Moves without approval: open gripper, retract arm, then the base drives{drive != null ? ` ≈ ${drive.toFixed(2)} m forward` : ""} to the
          pregrasp pose. The first approval is at align_refined_grasp.
        </div>
      </>
    );
  }
  if (arm.kind === "return") {
    const release = arm.args.release_cup !== false;
    return (
      <>
        <h3>Return to start?</h3>
        {r ? (
          <div className="arm-split">
            <div className="facts">
              <div>
                <b>{r.home_distance_m.toFixed(2)} m</b>
                <span>to start</span>
              </div>
              <div>
                <b>
                  {r.home_bearing_deg > 0 ? "+" : ""}
                  {r.home_bearing_deg.toFixed(0)}°
                </b>
                <span>bearing</span>
              </div>
              <div>
                <b>
                  {r.home_turn_deg > 0 ? "+" : ""}
                  {r.home_turn_deg.toFixed(1)}°
                </b>
                <span>final turn</span>
              </div>
            </div>
            <TopDown now={r.pose} target={{ x: 0, y: 0, theta: 0 }} label="path back to the start spot" />
          </div>
        ) : (
          <div className="moves">Joint state unknown: the distance to the start spot can't be shown.</div>
        )}
        {ex.holding_cup && (
          <div className="choice" role="radiogroup" aria-label="Held cup">
            <label>
              <input type="radio" checked={release} onChange={() => setArm({ ...arm, args: { ...arm.args, release_cup: true } })} />
              <span>
                Release the cup first <small>opens the gripper where it is, detaches detected_cup</small>
              </span>
            </label>
            <label>
              <input type="radio" checked={!release} onChange={() => setArm({ ...arm, args: { ...arm.args, release_cup: false } })} />
              <span>
                Keep holding <small>plans home with the cup attached</small>
              </span>
            </label>
          </div>
        )}
        <div className="moves">
          Sends <span className="mono">home</span>: the base drives to (0, 0, 0) and the arm goes upright. The plan comes back for approval before
          anything moves.
        </div>
      </>
    );
  }
  if (arm.kind === "undo_step" || arm.kind === "undo_sequence") {
    const items = arm.kind === "undo_sequence" ? ex.undo_stack : ex.undo_stack.filter((u) => u.reversible).slice(0, 1);
    return (
      <>
        <h3>{arm.kind === "undo_sequence" ? `Undo ${items.filter((u) => u.reversible).length} steps?` : `Undo ${items[0]?.step ?? "last step"}?`}</h3>
        <div className="undo-list">
          {items.map((u, i) => (
            <div key={u.undo_id} className={u.reversible ? "" : "muted"}>
              <span className="mono">
                {i + 1}. {u.step}
              </span>
              <span>{!u.reversible ? "detect: nothing to undo" : u.type === "gripper" ? `gripper back to ${u.gripper_before.toFixed(0)}°` : "reverse replay"}</span>
            </div>
          ))}
        </div>
        <div className="moves">
          Newest first. Each step is re-checked against the current planning scene and comes back for approval. If the robot has drifted from a step's
          recorded endpoint, the console says so and plans a fresh move to the pre-step pose instead.
        </div>
      </>
    );
  }
  if (arm.kind === "resume") {
    const res = ex.resumable;
    return (
      <>
        <h3>Retry {res?.command} from step {res?.step_index}?</h3>
        <div className="moves">
          Re-detects the target, then continues from <span className="mono">{res?.step}</span>. Each motion that needs approval comes back for approval.
        </div>
      </>
    );
  }
  return (
    <>
      <h3>{arm.kind === "gripper_open" ? "Open the gripper?" : "Close the gripper?"}</h3>
      <div className="moves">
        {arm.kind === "gripper_open"
          ? ex.holding_cup
            ? "The gripper opens and releases the cup where it is (detaches detected_cup)."
            : "The gripper opens."
          : "The gripper closes."}
      </div>
    </>
  );
}

export function ArmConfirm({ snap, arm, setArm }: { snap: Snapshot; arm: Arm; setArm: (a: Arm | null) => void }) {
  const name = ACTION[arm.kind];
  const ok = !!snap.allowed[name]?.allowed;
  return (
    <>
      <button
        className="btn motion lg"
        disabled={!ok}
        title={snap.allowed[name]?.reason ?? ""}
        onClick={() => {
          setArm(null);
          void act(name, arm.args);
        }}
      >
        <Move size={18} aria-hidden /> {confirmLabel(arm, snap)}
      </button>
      <button className="btn" onClick={() => setArm(null)}>
        Cancel
      </button>
      {!ok && <div className="why-line">{snap.allowed[name]?.reason}</div>}
    </>
  );
}

// Manual motion lives in the bay too, so read-only controls and motion are never neighbours.
export function ManualMotion({ snap, setArm, inspected }: { snap: Snapshot; setArm: (a: Arm | null) => void; inspected?: boolean }) {
  const a = snap.allowed;
  const next = snap.recovery?.undo_next;
  const b = (kind: ArmKind, name: string, label: string, icon: JSX.Element) => (
    <button className="btn motion sm" disabled={!a[name]?.allowed} title={a[name]?.reason ?? ""} onClick={() => setArm({ kind, args: inspected ? { inspected } : {} })}>
      {icon} {label}
    </button>
  );
  return (
    <div className="manual">
      {b("return", "return_to_start", "Return to start", <Home size={14} aria-hidden />)}
      {b("undo_step", "undo_step", next ? `Undo ${next.step}` : "Undo last step", <Undo2 size={14} aria-hidden />)}
      {b("gripper_open", "gripper_open", "Open gripper", <Hand size={14} aria-hidden />)}
      {b("gripper_close", "gripper_close", "Close gripper", <RotateCcw size={14} aria-hidden style={{ transform: "scaleX(-1)" }} />)}
    </div>
  );
}
