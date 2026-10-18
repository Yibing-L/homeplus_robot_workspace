import { useEffect, useState } from "react";
import { act, useConsole } from "../store";
import type { Snapshot } from "../types";
import { fmtAge, fmtClock, useSinceSnapshot } from "../util";

export function StopButton() {
  const stop = useConsole((s) => s.snap?.stop ?? null);
  const local = useConsole((s) => s.lastResult.stop);
  const [shownUntil, setShownUntil] = useState(0);
  const press = () => {
    setShownUntil(Date.now() + 8000);
    void act("stop");
  };
  useEffect(() => {
    // Esc is STOP everywhere, even inside inputs and drawers (§6.1).
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        e.preventDefault();
        press();
      }
    };
    window.addEventListener("keydown", onKey, true);
    return () => window.removeEventListener("keydown", onKey, true);
  }, []);
  const [, tick] = useState(0);
  useEffect(() => {
    if (shownUntil <= Date.now()) return;
    const id = window.setTimeout(() => tick((n) => n + 1), shownUntil - Date.now() + 10);
    return () => window.clearTimeout(id);
  }, [shownUntil]);
  // Server verification wins; the POST result only covers the console being unreachable.
  const show = shownUntil > Date.now() || stop?.status === "not_confirmed";
  const status = local?.status === "failed" ? "not_confirmed" : stop?.status ?? "sent";
  const detail = local?.status === "failed" ? "Console unreachable: STOP not sent. Use the physical E-stop." : stop?.detail ?? "STOP sent";
  return (
    <div className="stopbay">
      <button className="stop" onClick={press} aria-label="Stop the robot (Escape)">
        STOP <kbd>Esc</kbd>
      </button>
      {show && (
        <div className={`stop-result ${status}`} role="status" aria-live="assertive">
          {detail}
        </div>
      )}
    </div>
  );
}

function Chip({ k, v, cls = "", title, onClick }: { k: string; v: string; cls?: string; title?: string; onClick?: () => void }) {
  const body = (
    <>
      <span className="dot" aria-hidden />
      <b>{k}</b>
      <span className="v">{v}</span>
    </>
  );
  return onClick ? (
    <button className={`chip ${cls}`} title={title} onClick={onClick}>
      {body}
    </button>
  ) : (
    <div className={`chip ${cls}`} title={title}>
      {body}
    </div>
  );
}

// No native confirm(): it would block the page, and STOP must never be covered (§6.1).
function TakeControl({ holder }: { holder: string | null }) {
  const [confirming, setConfirming] = useState(false);
  useEffect(() => {
    if (!confirming) return;
    const id = window.setTimeout(() => setConfirming(false), 5000);
    return () => window.clearTimeout(id);
  }, [confirming]);
  if (holder && !confirming) return <Chip k="View only" v={`${holder} controls · take control`} cls="warn" onClick={() => setConfirming(true)} />;
  return (
    <Chip
      k={holder ? `Take control from ${holder}?` : "View only"}
      v={holder ? "confirm" : "take control"}
      cls="warn"
      onClick={() => {
        setConfirming(false);
        void act("take_control");
      }}
    />
  );
}

export function Header({ snap }: { snap: Snapshot | null }) {
  const { since, lost } = useSinceSnapshot();
  const operator = useConsole((s) => s.operator);
  if (!snap) {
    return (
      <header className="top">
        <StopButton />
        <div className="headwrap">
          <div className="headline">
            <span className="title unknown">Connecting…</span>
          </div>
        </div>
      </header>
    );
  }
  const r = snap.robot;
  const mp = snap.motor_power;
  const o = snap.origin;
  const p = snap.perception;
  const mpCls = mp.value === "suspect" ? "bad" : mp.value === "verified" ? "" : mp.value === "unknown" ? "unknown" : "warn";
  const oCls = o.status === "lost" ? "bad" : o.status === "confirmed" ? "" : "warn";
  const pCls = p.severity === "bad" ? "bad" : p.severity === "warn" ? "warn" : p.severity === "unknown" ? "unknown" : "";
  const ctrl = snap.control;
  return (
    <header className="top">
      <StopButton />
      <div className="headwrap" style={lost ? { opacity: 0.55 } : undefined}>
        <div className="headline">
          <span className={`title ${lost ? "bad" : r.severity}`}>{lost ? "Console link lost" : r.title}</span>
          <span className="detail" title={r.detail}>
            {lost ? `no snapshot for ${fmtAge(since)} · use the physical E-stop` : r.detail}
          </span>
          <span className="since">{lost ? "values frozen" : `since ${fmtClock(r.since_wall, true)}`}</span>
        </div>
        <div className="chips">
          <Chip k="Perception" v={p.title.toLowerCase()} cls={pCls} title={p.detail} />
          <Chip
            k="Motors"
            v={mp.value === "verified" && mp.verified ? `verified ${fmtClock(mp.verified.at)}` : mp.value}
            cls={mpCls}
            title={mp.reasons.map((x) => x.text).join(" ") || mp.verified?.how}
          />
          <Chip
            k="Origin"
            v={o.status === "confirmed" ? `confirmed ${fmtClock(o.at)}` : o.status}
            cls={oCls}
            title={o.status === "confirmed" ? `by ${o.by}${o.note ? ` · ${o.note}` : ""}` : o.lost_reason ?? "not confirmed this boot"}
          />
          {snap.executor.holding_cup && <Chip k="Gripper" v="holding cup" cls="warn" title="detected_cup is attached to grasp_tcp" />}
          {snap.estop_declaration && (
            <Chip
              k="E-stop"
              v={`declared ${fmtClock(snap.estop_declaration.at)}`}
              cls="declared"
              title={`Declared by ${snap.estop_declaration.by}. The console can't observe the E-stop.`}
            />
          )}
          <span className="spacer" />
          {ctrl.you ? (
            <Chip k="Link" v={`you · ${fmtAge(since)}`} title={`Controlling as ${operator}`} />
          ) : (
            <TakeControl holder={ctrl.held ? ctrl.holder : null} />
          )}
          {snap.mode === "sim" && <Chip k="Sim" v="" cls="sim" title="Signal-level simulation: no robot connected" />}
          {snap.degraded && <Chip k="Degraded" v="status text" cls="warn" title={snap.degraded} />}
        </div>
      </div>
    </header>
  );
}
