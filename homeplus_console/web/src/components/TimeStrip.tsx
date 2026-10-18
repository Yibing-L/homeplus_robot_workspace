import { useId } from "react";

// The flowsheet's signature: one tick per 0.5 s receipt bin over the last 60 s, against a "now" edge.
// When a signal goes quiet for longer than its freshness threshold, the gap since its last tick
// is hatched and an amber onset rule marks when it stopped.

const BINS = 120;
const BIN_SEC = 0.5;

export function TimeStrip({ strip, staleAfter, label }: { strip: string; staleAfter: number; label?: string }) {
  const bins = strip.length === BINS ? strip : "0".repeat(BINS);
  let last = -1;
  for (let i = BINS - 1; i >= 0; i--) {
    if (bins[i] !== "0") {
      last = i;
      break;
    }
  }
  const silentSec = last < 0 ? Infinity : (BINS - 1 - last) * BIN_SEC;
  const gap = silentSec > staleAfter;
  const gapStart = last < 0 ? 0 : last + 1;
  const ticks: JSX.Element[] = [];
  for (let i = 0; i < BINS; i++) {
    const c = bins.charCodeAt(i) - 48;
    if (c <= 0) continue;
    const h = Math.min(1, 0.45 + c * 0.12);
    ticks.push(<rect key={i} x={i + 0.2} y={(1 - h) * 10} width={0.55} height={h * 20} />);
  }
  const id = useId().replace(/:/g, "");
  return (
    <span className="strip" role="img" aria-label={label ?? (gap ? `silent ${Number.isFinite(silentSec) ? silentSec.toFixed(0) + " s" : "since start"}` : "receiving")}>
      <svg viewBox={`0 0 ${BINS} 20`} preserveAspectRatio="none">
        <defs>
          <pattern id={id} width="3" height="3" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
            <rect width="1.1" height="3" className="gapfill" />
          </pattern>
        </defs>
        {[24, 48, 72, 96].map((x) => (
          <line key={x} x1={x} x2={x} y1={0} y2={20} className="g" strokeWidth={1} vectorEffect="non-scaling-stroke" />
        ))}
        <g className="t">{ticks}</g>
        {gap && (
          <>
            <rect x={gapStart} y={0} width={BINS - gapStart} height={20} fill={`url(#${id})`} />
            {last >= 0 && <line x1={gapStart} x2={gapStart} y1={0} y2={20} className="onset" strokeWidth={1.5} vectorEffect="non-scaling-stroke" />}
          </>
        )}
        <line x1={BINS} x2={BINS} y1={0} y2={20} className="now" strokeWidth={1.5} vectorEffect="non-scaling-stroke" />
      </svg>
    </span>
  );
}
