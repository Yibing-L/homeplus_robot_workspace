import { useState } from "react";
import { act, useConsole } from "../store";
import type { Component, Snapshot } from "../types";
import { fmtAge, fmtClock, signed, useSinceSnapshot } from "../util";
import { TimeStrip } from "./TimeStrip";

const STALE_AFTER: Record<string, number> = {
  "hp-tf": 1, "hp-camera-rx": 1, "hp-vision": 10, "hp-moveit": 1.5, "hp-bridge": 1, d455: 1, gpu: 10,
};

function UnitButtons({ c, snap }: { c: Component; snap: Snapshot }) {
  const pending = useConsole((s) => s.pending);
  if (!c.unit) return <div className="muted">Observed only: the console doesn't run this process.</div>;
  const bridge = c.unit === "hp-bridge";
  const a = snap.allowed;
  const tracked = [...snap.tracked_actions].reverse().find((t) => (t.args as { unit?: string }).unit === c.unit);
  const btn = (name: "unit_start" | "unit_stop" | "unit_restart", label: string) => {
    const blockedBridge = bridge && name !== "unit_stop";
    const ok = a[name]?.allowed && !blockedBridge;
    return (
      <button
        className="btn sm"
        disabled={!ok || !!pending[name]}
        title={blockedBridge ? "Starting the bridge resets the Arduino and odometry: use Re-zero" : a[name]?.reason ?? ""}
        onClick={() => act(name, { unit: c.unit })}
      >
        {label}
      </button>
    );
  };
  return (
    <>
      <div className="btns">
        {btn("unit_start", "Start")}
        {btn("unit_stop", bridge ? "Stop (sends STOP first)" : "Stop")}
        {btn("unit_restart", "Restart")}
        {bridge && (
          <button className="btn sm" disabled={!a.flow_rezero_start?.allowed} onClick={() => act("flow_rezero_start", { reason: "bridge restart requested" })}>
            Re-zero…
          </button>
        )}
      </div>
      {tracked && <div className={`result-line ${tracked.status}`}>{tracked.detail}</div>}
    </>
  );
}

function Row({ c, snap }: { c: Component; snap: Snapshot }) {
  const [open, setOpen] = useState(false);
  const o = c.observed;
  const value = c.value ?? "—";
  return (
    <div className={`hrow ${c.health}`}>
      <button
        onClick={() => setOpen((v) => !v)}
        aria-expanded={open}
        title={`${o.source} · last ${fmtAge(o.age)} ago · ${o.rate_hz.toFixed(1)} Hz (10 s)`}
      >
        <span className={`mark ${c.health === "ok" ? "ok" : c.health === "starting" ? "starting" : c.health === "silent" ? "silent" : c.health === "down" ? "down" : "unknown"}`} aria-hidden />
        <span className="name">
          {c.label}
          <span className="sub">{c.health === "ok" ? c.unit ?? "observed" : c.detail}</span>
        </span>
        <span className="val mono">
          {c.health === "ok" ? value : "—"}
          <small> · {o.age !== null ? fmtAge(o.age) : "never"}</small>
        </span>
        <TimeStrip strip={o.strip} staleAfter={STALE_AFTER[c.id] ?? 2} />
      </button>
      {open && (
        <div className="hrow-more">
          <div className="mono">
            {c.unit ? `${c.unit}: ${c.unit_state ?? "?"}/${c.sub_state ?? "?"}` : "no unit"}
            {c.pid ? ` · pid ${c.pid}` : ""}
            {c.uptime_s != null ? ` · up ${fmtAge(c.uptime_s)}` : ""}
            {c.restarts ? ` · ${c.restarts} restart${c.restarts > 1 ? "s" : ""}` : ""}
          </div>
          <div className="mono muted">
            observed {o.source}: {o.fresh ? "fresh" : "stale"}, last {fmtAge(o.age)} ago
          </div>
          {c.last_error && (
            <div className="err">
              {fmtClock(c.last_error.t, true)} {c.last_error.text}
            </div>
          )}
          <UnitButtons c={c} snap={snap} />
        </div>
      )}
    </div>
  );
}

export function Health({ snap }: { snap: Snapshot }) {
  const { lost } = useSinceSnapshot();
  const j = snap.joints;
  const vals = j.values ?? j.last_values;
  const ok = snap.components.filter((c) => c.health === "ok").length;
  return (
    <section className="panel" aria-label="Health">
      <div className="panel-head">
        <h2>Health</h2>
        <span className="label">
          {lost ? "unknown · link lost" : `${ok}/${snap.components.length} live · −60 s → now`}
        </span>
      </div>
      <div className="panel-body">
        {snap.components.map((c) => (
          <Row key={c.id} c={c} snap={snap} />
        ))}
        <div className={`joints ${j.fresh ? "" : "unknown"}`}>
          <div className="label" style={{ display: "flex", justifyContent: "space-between" }}>
            <span>Joints {j.fresh ? "" : "· unknown"}</span>
            <span className="muted">
              {j.fresh ? `${j.ready ? "ready" : "not ready"} · ${j.arm_still ? "still" : "moving"}` : `last seen ${fmtAge(j.age)} ago`}
            </span>
          </div>
          <div className="grid">
            {(
              [
                ["Base x", vals && signed(vals.base_x, 2, " m")],
                ["Base y", vals && signed(vals.base_y, 2, " m")],
                ["Base θ", vals && signed(vals.base_theta, 1, "°")],
                ["Shoulder", vals && `${vals.joint_shoulder.toFixed(0)}°`],
                ["Elbow", vals && `${vals.joint_elbow.toFixed(0)}°`],
                ["Gripper", vals && `${vals.joint_grip_L.toFixed(0)}°`],
              ] as [string, string | null][]
            ).map(([k, v]) => (
              <div key={k}>
                <div className="k">{k}</div>
                <div className="v">{j.fresh ? v ?? "—" : "—"}</div>
              </div>
            ))}
          </div>
          {!j.fresh && vals && (
            <div className="muted" style={{ fontSize: 12.5 }}>
              Last seen {fmtAge(j.age)} ago: x {vals.base_x.toFixed(2)} m, elbow {vals.joint_elbow.toFixed(0)}°
            </div>
          )}
        </div>
      </div>
    </section>
  );
}
