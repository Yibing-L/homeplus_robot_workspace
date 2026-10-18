import { X } from "lucide-react";
import { useMemo, useState } from "react";
import { simFault } from "../store";
import type { Snapshot } from "../types";
import { fmtClock } from "../util";

export function LogsDrawer({ snap, onClose }: { snap: Snapshot; onClose: () => void }) {
  const [node, setNode] = useState("all");
  const [important, setImportant] = useState(true);
  const [q, setQ] = useState("");
  const nodes = useMemo(() => Array.from(new Set(snap.logs.map((l) => l.node))).sort(), [snap.logs]);
  const lines = snap.logs.filter(
    (l) => (node === "all" || l.node === node) && (!important || l.important) && (!q || l.text.toLowerCase().includes(q.toLowerCase())),
  );
  return (
    <div className="drawer" onClick={onClose}>
      <div className="sheet" onClick={(e) => e.stopPropagation()} role="dialog" aria-label="Logs">
        <div className="tools">
          <b style={{ marginRight: "auto" }}>Logs</b>
          <select value={node} onChange={(e) => setNode(e.target.value)} aria-label="Node">
            <option value="all">All nodes</option>
            {nodes.map((n) => (
              <option key={n}>{n}</option>
            ))}
          </select>
          <input type="text" placeholder="Filter text" value={q} onChange={(e) => setQ(e.target.value)} aria-label="Filter text" />
          <label style={{ display: "flex", gap: 6, alignItems: "center", fontSize: 14 }}>
            <input type="checkbox" checked={important} onChange={(e) => setImportant(e.target.checked)} /> Important only
          </label>
          <a className="btn sm" href="/api/logs?since=0" download="homeplus_console_logs.json">
            Download
          </a>
          <button className="btn sm" onClick={onClose} aria-label="Close logs">
            <X size={16} />
          </button>
        </div>
        <div className="loglist">
          {lines.length === 0 && <div className="muted">No lines match.</div>}
          {lines.map((l) => (
            <div key={l.seq} className={l.level}>
              <span>{fmtClock(l.t_wall, true)}</span>
              <span className="node">{l.node}</span>
              <span>{l.text}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

// Sim mode only: reproduce each real failure from UI_PLAN_PROMPT.md.
const TOGGLES: [string, string, string][] = [
  ["motor_power", "Motor power (E-stop released)", "E-stop pressed / power off"],
  ["usb_connected", "Arduino serial link up", "link down"],
  ["sender_alive", "Camera sender alive", "Sender dead"],
  ["sender_frozen", "Sender frozen (same frame)", ""],
  ["d455_alive", "D455 alive", "D455 dead"],
  ["gpu_up", "GPU server up", "GPU down"],
  ["zero_supported", "Firmware supports ZERO (§7.10)", ""],
];
const SCENARIOS = ["good", "two_targets", "none", "stale_frames", "garbage_z", "high_first"];

export function SimDrawer({ snap, onClose }: { snap: Snapshot; onClose: () => void }) {
  const f = (snap.sim ?? {}) as Record<string, unknown>;
  return (
    <div className="drawer" onClick={onClose}>
      <div className="sheet sim-panel" style={{ width: 420 }} onClick={(e) => e.stopPropagation()} role="dialog" aria-label="Sim faults">
        <div className="tools">
          <b style={{ marginRight: "auto" }}>Sim faults</b>
          <button className="btn sm" onClick={onClose} aria-label="Close sim faults">
            <X size={16} />
          </button>
        </div>
        <div className="group">
          <h4>Hardware</h4>
          {TOGGLES.map(([k, on]) => (
            <label key={k} style={{ display: "flex", gap: 8, alignItems: "center", fontSize: 14 }}>
              <input type="checkbox" checked={!!f[k]} onChange={(e) => simFault(k, e.target.checked)} /> {on}
            </label>
          ))}
        </div>
        <div className="group">
          <h4>Detector scenario</h4>
          <div className="row">
            {SCENARIOS.map((s) => (
              <button key={s} className={`btn sm ${f.detector_scenario === s ? "on" : ""}`} onClick={() => simFault("detector_scenario", s)}>
                {s}
              </button>
            ))}
          </div>
        </div>
        <div className="group">
          <h4>One-shot</h4>
          <div className="row">
            <button className="btn sm" onClick={() => simFault("push_base", 0.05)}>Push robot 5 cm</button>
            <button className="btn sm" onClick={() => simFault("reset_arduino", true)}>Arduino reset (firmware reboot)</button>
            <button className="btn sm" onClick={() => simFault("fail_unit", "hp-moveit")}>Crash hp-moveit</button>
            <button className="btn sm" onClick={() => simFault("link_pause", 6)}>Drop console link 6 s</button>
          </div>
        </div>
        <div className="group muted" style={{ fontSize: 13 }}>
          Faults act on the simulated robot only. Updated {fmtClock(snap.server_wall, true)}.
        </div>
      </div>
    </div>
  );
}
