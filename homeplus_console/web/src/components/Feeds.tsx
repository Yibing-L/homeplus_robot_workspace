import { useEffect, useRef, useState } from "react";
import type { Feed, Perception, Snapshot } from "../types";
import { fmtAge, useSinceSnapshot } from "../util";
import { Plan3D } from "./Plan3D";
import { TimeStrip } from "./TimeStrip";

// Frames are fetched per sequence number, only when the snapshot says a newer one exists and the
// previous fetch has finished, so a slow link drops frames instead of queueing them.
function useFrame(feed: Feed): string | null {
  const [url, setUrl] = useState<string | null>(null);
  const loading = useRef(false);
  const shown = useRef<number | null>(null);
  useEffect(() => {
    if (feed.seq == null || feed.seq === shown.current || loading.current) return;
    loading.current = true;
    const next = `/img/${feed.id}.jpg?seq=${feed.seq}`;
    const img = new Image();
    img.onload = () => {
      shown.current = feed.seq;
      loading.current = false;
      setUrl(next);
    };
    img.onerror = () => {
      loading.current = false;
    };
    img.src = next;
  }, [feed.id, feed.seq]);
  return url;
}

function FeedTile({ feed, big, onClick }: { feed: Feed; big?: boolean; onClick?: () => void }) {
  const url = useFrame(feed);
  const { since, lost } = useSinceSnapshot();
  const age = feed.age == null ? null : feed.age + (lost ? 0 : since);
  const status = lost ? "stale" : feed.status;
  const veil =
    status === "off" ? (
      <div className="veil off">
        <b>NO FRAMES</b>
        <span>{feed.topic ?? "feed"} has not published</span>
      </div>
    ) : status === "stale" ? (
      <div className="veil">
        <b>STALE</b>
        <span>{lost ? "console link lost" : feed.note ?? `last frame ${fmtAge(age)} ago`}</span>
      </div>
    ) : status === "frozen" ? (
      <div className="veil">
        <b>FROZEN</b>
        <span>{feed.note ?? `frames identical for ${fmtAge(feed.frozen_sec)}: sender hung`}</span>
      </div>
    ) : null;
  return (
    <figure className={`feed is-${status} ${big ? "big" : ""}`} style={{ margin: 0, cursor: onClick ? "zoom-in" : undefined }}
      aria-label={`${feed.label}: ${status}`} onClick={onClick} title={big ? "click to enlarge" : "click to show large"}>
      <div className="frame">
        {url && <img src={url} alt="" draggable={false} />}
        <span className="tag">{feed.label}</span>
        {feed.kind === "depth" && feed.depth_range_m && status !== "off" && (
          <div className="scale" aria-label={`depth ${feed.depth_range_m[0]} to ${feed.depth_range_m[1]} m`}>
            <span>{feed.depth_range_m[0].toFixed(2)}</span>
            <span className="bar" />
            <span>{feed.depth_range_m[1].toFixed(2)} m</span>
          </div>
        )}
        {veil}
      </div>
      <figcaption className="cap" title={`${feed.topic ?? ""} · frame #${feed.seq ?? "—"}`}>
        <span className="age">{fmtAge(age)}</span>
        <span className="mono muted">
          {status === "stale" || status === "off" ? "— fps" : `${feed.fps.toFixed(1)} fps`}
          {feed.kind === "overlay" && feed.source_age != null ? ` · src ${fmtAge(feed.source_age)}` : ""}
        </span>
        <TimeStrip strip={feed.strip} staleAfter={feed.stale_after} />
      </figcaption>
    </figure>
  );
}

function ZBand({ p }: { p: Perception }) {
  const w = p.window;
  const [lo, hi] = w.z_band;
  const min = lo - 0.15;
  const max = hi + 0.15;
  const x = (z: number) => ((Math.min(max, Math.max(min, z)) - min) / (max - min)) * 150;
  return (
    <svg className="zband" viewBox="0 0 150 22" aria-label={`z band ${lo}–${hi} m`}>
      <title>{`z band ${lo.toFixed(2)}–${hi.toFixed(2)} m; dots are the last detections, red when outside`}</title>
      <rect className="band" x={x(lo)} y={4} width={x(hi) - x(lo)} height={14} strokeWidth={0.8} />
      <line className="axis" x1={0} x2={150} y1={11} y2={11} />
      {w.samples.map((s, i) => (
        <circle key={i} className={`s ${s.z < lo || s.z > hi ? "out" : ""}`} cx={x(s.z)} cy={11} r={2.6} opacity={Math.max(0.25, 1 - s.age / 20)} />
      ))}
    </svg>
  );
}

function Detection({ p }: { p: Perception }) {
  const d = p.diagnostics;
  const w = p.window;
  const cls = p.severity === "bad" ? "badline" : p.severity === "warn" ? "warnline" : "";
  const untrusted = ["CAMERA_DOWN", "CAMERA_FROZEN", "DETECTOR_OLD_FRAMES"].includes(p.state);
  return (
    <div className={`detect ${untrusted ? "untrusted" : ""}`}>
      <span className="stage" title={`detector ${d.backend ?? ""} · cycle ${d.cycle_sec ?? "—"} s`}>
        {d.stage ? `${d.stage}: ${d.reason}` : "no diagnostics"}
        <span className="muted"> · {fmtAge(d.age)} ago</span>
      </span>
      <span className="win">
        {p.severity !== "ok" ? (
          <span className={cls}>
            {p.title}: {p.detail}
          </span>
        ) : (
          <>
            <span>
              <b>{w.consecutive}</b> consecutive (need {w.required})
            </span>
            <span>
              spread <b>{w.spread_cm ?? "—"} cm</b>
            </span>
            <span>
              z <b>{w.z?.toFixed(3) ?? "—"} m</b>
            </span>
          </>
        )}
      </span>
      <ZBand p={p} />
    </div>
  );
}

export function Feeds({ snap }: { snap: Snapshot }) {
  const [focus, setFocus] = useState<string | null>(null);
  const [full, setFull] = useState(false);
  const main = snap.feeds.find((f) => f.id === focus) ?? snap.feeds[0];
  const rest = snap.feeds.filter((f) => f !== main);
  const [tab, setTab] = useState<"cameras" | "plan">("cameras");
  const pid = snap.approval?.plan_id ?? null;
  const shownFor = useRef<number | null>(null);
  useEffect(() => {
    // bring the plan forward once per new approval; the operator can switch back
    if (pid != null && shownFor.current !== pid) {
      shownFor.current = pid;
      setTab("plan");
    }
  }, [pid]);
  const plan = snap.approval?.preview ?? snap.plan_preview;
  const { lost } = useSinceSnapshot();
  return (
    <section className="panel" aria-label="Cameras and plan">
      <div className="viewtabs" role="tablist">
        <button role="tab" aria-selected={tab === "cameras"} onClick={() => setTab("cameras")}>
          Cameras
        </button>
        <button role="tab" aria-selected={tab === "plan"} onClick={() => setTab("plan")}>
          Plan 3D{snap.approval && <span className="pip" aria-label="plan awaiting approval" />}
        </button>
        <span className="hint">
          {tab === "plan"
            ? "drag to orbit · scroll to zoom"
            : lost
              ? "feeds unknown: link lost"
              : `${snap.feeds.filter((f) => f.status === "live").length}/${snap.feeds.length} feeds live`}
        </span>
      </div>
      {tab === "cameras" ? (
        <div className="feeds">
          <FeedTile feed={main} big onClick={() => setFull(true)} />
          <div className="small">
            {rest.map((f) => (
              <FeedTile key={f.id} feed={f} onClick={() => setFocus(f.id)} />
            ))}
          </div>
          {full && (
            <div className="feed-full" role="dialog" aria-label={`${main.label} enlarged`} onClick={() => setFull(false)}>
              <FeedTile feed={main} big />
              <span className="hint">click anywhere to close</span>
            </div>
          )}
        </div>
      ) : (
        <Plan3D plan={plan} planAge={snap.approval ? null : snap.plan_preview_age} now={snap.joints.si} scene={snap.scene ?? null} />
      )}
      <Detection p={snap.perception} />
    </section>
  );
}
