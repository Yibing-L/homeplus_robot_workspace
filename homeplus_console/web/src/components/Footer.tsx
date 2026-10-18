import { Eraser, FlaskConical, ListTree, MapPin } from "lucide-react";
import { act } from "../store";
import type { Snapshot } from "../types";
import { fmtClock } from "../util";

// Read-only and setup controls only. Everything that moves the robot lives in the action bar's motion bay.
export function Footer({ snap, onLogs, onSim }: { snap: Snapshot; onLogs: () => void; onSim: () => void }) {
  const a = snap.allowed;
  const recent = [...snap.logs].reverse().filter((l) => l.important).slice(0, 2);
  return (
    <footer className="bottom">
      <button
        className="btn"
        disabled={!a.flow_set_start?.allowed}
        title="Zero odometry where the robot stands (ZERO), or push it to a new start spot first"
        onClick={() => act("flow_set_start", { reason: "operator: set start here" })}
      >
        <MapPin size={16} aria-hidden /> Set start here…
      </button>
      <button className="btn" disabled={!a.clear_octomap?.allowed} title={a.clear_octomap?.reason ?? ""} onClick={() => act("clear_octomap")}>
        <Eraser size={16} aria-hidden /> Clear octomap
      </button>
      <span className="sep" />
      <button className="btn" onClick={onLogs}>
        <ListTree size={15} aria-hidden /> Logs
      </button>
      <div className="logline" aria-live="off">
        {recent.map((l) => (
          <span key={l.seq} className={`text ${l.level}`} title={l.text}>
            <span className="mono">{fmtClock(l.t_wall, true)}</span> <span className="node">{l.node}</span> {l.text}
          </span>
        ))}
      </div>
      {snap.mode === "sim" && (
        <button className="btn sm" onClick={onSim}>
          <FlaskConical size={15} aria-hidden /> Sim faults
        </button>
      )}
    </footer>
  );
}
