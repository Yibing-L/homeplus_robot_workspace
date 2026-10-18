// Top-down view in the odometry frame: the start spot (0,0), the robot footprint now, the target
// pose, and the path between them. Exact geometry from joint states; no rendering guesswork.

type Pose = { x: number; y: number; theta: number }; // metres, metres, degrees

const FOOT = { l: 0.5, w: 0.42 }; // base footprint, metres

function Footprint({ p, s, cls }: { p: Pose; s: (x: number, y: number) => [number, number]; cls: string }) {
  const [cx, cy] = s(p.x, p.y);
  const scale = s(1, 0)[0] - s(0, 0)[0];
  return (
    <g transform={`translate(${cx} ${cy}) rotate(${-p.theta})`} className={cls}>
      <rect x={(-FOOT.l / 2) * scale} y={(-FOOT.w / 2) * scale} width={FOOT.l * scale} height={FOOT.w * scale} rx={3} />
      <line x1={0} y1={0} x2={(FOOT.l / 2) * scale} y2={0} strokeWidth={2} />
    </g>
  );
}

export function TopDown({ now, target, label }: { now: Pose | null; target: Pose; label?: string }) {
  const W = 300;
  const H = 150;
  const pts = [target, ...(now ? [now] : [])];
  const xs = pts.map((p) => p.x);
  const ys = pts.map((p) => p.y);
  const span = Math.max(1.2, Math.max(...xs) - Math.min(...xs) + 0.9, (Math.max(...ys) - Math.min(...ys) + 0.9) * 2);
  const scale = W / span;
  const cxm = (Math.max(...xs) + Math.min(...xs)) / 2;
  const cym = (Math.max(...ys) + Math.min(...ys)) / 2;
  // odometry x forward → right, y left → up
  const s = (x: number, y: number): [number, number] => [W / 2 + (x - cxm) * scale, H / 2 - (y - cym) * scale];
  const [tx, ty] = s(target.x, target.y);
  const grid = [];
  for (let gx = Math.floor((cxm - span / 2) * 2) / 2; gx <= cxm + span / 2; gx += 0.5) {
    const [x] = s(gx, 0);
    grid.push(<line key={`x${gx}`} x1={x} x2={x} y1={0} y2={H} />);
  }
  return (
    <svg className="topdown" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={label ?? "top-down path"}>
      <g className="grid">{grid}</g>
      <g className="tape">
        <rect x={tx - 16} y={ty - 3} width={32} height={6} />
      </g>
      <Footprint p={target} s={s} cls="ghost" />
      {now && (
        <>
          <line className="path" x1={s(now.x, now.y)[0]} y1={s(now.x, now.y)[1]} x2={tx} y2={ty} />
          <Footprint p={now} s={s} cls="robot" />
        </>
      )}
      <text x={tx} y={ty + 22} textAnchor="middle" className="cap">
        start (0, 0)
      </text>
      <text x={W - 4} y={H - 6} textAnchor="end" className="cap">
        0.5 m grid
      </text>
    </svg>
  );
}

export function headingTo(now: Pose, target: Pose): { dist: number; bearing: number; turn: number } {
  const dx = target.x - now.x;
  const dy = target.y - now.y;
  const dist = Math.hypot(dx, dy);
  const bearing = (Math.atan2(dy, dx) * 180) / Math.PI; // in odometry frame
  const turn = target.theta - now.theta;
  return { dist, bearing: bearing - now.theta, turn };
}
