import { useEffect, useState } from "react";
import { LINK_LOST_MS, useConsole } from "./store";

/** Re-render on an interval; ages and countdowns advance between snapshots. */
export function useNow(ms = 200): number {
  const [now, setNow] = useState(() => performance.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(performance.now()), ms);
    return () => window.clearInterval(id);
  }, [ms]);
  return now;
}

/** Seconds since the snapshot arrived, or null once the link is lost (values are frozen). */
export function useSinceSnapshot(): { since: number; lost: boolean } {
  const now = useNow(200);
  const receivedAt = useConsole((s) => s.receivedAt);
  const has = useConsole((s) => s.snap !== null);
  const since = (now - receivedAt) / 1000;
  return { since: Math.max(0, since), lost: !has || since * 1000 > LINK_LOST_MS };
}

export function fmtAge(sec: number | null | undefined): string {
  if (sec === null || sec === undefined || !Number.isFinite(sec)) return "—";
  if (sec < 10) return `${sec.toFixed(1)} s`;
  if (sec < 120) return `${Math.round(sec)} s`;
  if (sec < 7200) return `${Math.round(sec / 60)} min`;
  return `${(sec / 3600).toFixed(1)} h`;
}

export function fmtClock(wall: number | null | undefined, seconds = false): string {
  if (!wall) return "—";
  const d = new Date(wall * 1000);
  const p = (n: number) => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}${seconds ? ":" + p(d.getSeconds()) : ""}`;
}

export function signed(v: number, digits = 2, unit = ""): string {
  const s = v.toFixed(digits);
  if (Number(s) === 0) return (0).toFixed(digits) + unit;
  return (v > 0 ? "+" : "−") + s.replace("-", "") + unit;
}

export const JOINT_LABEL: Record<string, string> = {
  joint_shoulder: "Shoulder",
  joint_elbow: "Elbow",
  joint_wrist: "Wrist",
  joint_hand: "Hand",
  joint_grip_L: "Gripper",
};
