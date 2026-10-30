import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { PlanPreview, SceneView, SceneVoxels } from "../types";
import { fmtAge } from "../util";
import { HEIGHT_GRADIENT, SC, buildCollision, buildPrim, buildVoxels, disposeTree } from "./scene3d";

// The MoveIt plan in 3D, from the robot's own URDF and meshes (served by the console backend):
//   solid  = the robot now (fresh /hardware/joint_states only; hidden when joints are stale)
//   blue   = the planned motion, scrubbable / looping
//   line   = the TCP path through the planned waypoints, cup marker = detected target
//   scene  = MoveIt's planning scene (toggles "OctoMap" / "Objects"): occupied voxels, collision objects,
//            the attached cup on its link, and the last collision point the executor reported
// No guesswork: every pose comes from joint values the backend received.

type Joints = Record<string, number>;

const C = { now: 0x6b727c, plan: 0x6ea8fe, path: 0x6ea8fe, cup: 0xff5c4d, grid: 0x2a2e34, gridMain: 0x353a42 };

function loadRobot(color: number, opacity: number): Promise<URDFRobot> {
  return new Promise((resolve, reject) => {
    const loader = new URDFLoader();
    loader.packages = { homeplus_urdf_description: "/api/robot/pkg" };
    fetch("/api/robot/urdf")
      .then((r) => (r.ok ? r.text() : Promise.reject(new Error(`URDF HTTP ${r.status}`))))
      .then((xml) => {
        const mat = new THREE.MeshStandardMaterial({
          color,
          transparent: opacity < 1,
          opacity,
          roughness: 0.75,
          metalness: 0.05,
          depthWrite: opacity >= 1,
        });
        loader.loadMeshCb = (path, manager, done) => {
          import("three/examples/jsm/loaders/STLLoader.js").then(({ STLLoader }) => {
            new STLLoader(manager).load(path, (geom) => done(new THREE.Mesh(geom, mat)), undefined, (e) => done(null as never, e as Error));
          });
        };
        const robot = loader.parse(xml);
        robot.userData.mat = mat;
        resolve(robot);
      })
      .catch(reject);
  });
}

function tint(robot: URDFRobot, mat: THREE.Material) {
  robot.traverse((c) => {
    if ((c as THREE.Mesh).isMesh && (c as THREE.Mesh).material !== mat) (c as THREE.Mesh).material = mat;
  });
}

function useToggle(key: string): [boolean, () => void] {
  const [on, setOn] = useState(() => localStorage.getItem(key) !== "0");
  return [
    on,
    () =>
      setOn((v) => {
        localStorage.setItem(key, v ? "0" : "1");
        return !v;
      }),
  ];
}

const shortId = (id: string) => id.replace(/^detected_/, "");

function setJoints(robot: URDFRobot | null, j: Joints | null) {
  if (!robot || !j) return;
  for (const [k, v] of Object.entries(j)) robot.joints[k]?.setJointValue(v);
}

type SceneParts = { root: THREE.Group; voxels?: THREE.Group; voxSeq?: number; objects?: THREE.Group; attached: THREE.Object3D[] };

export function Plan3D({ plan, planAge, now, scene }: { plan: PlanPreview | null; planAge: number | null; now: Joints | null; scene: SceneView | null }) {
  const host = useRef<HTMLDivElement>(null);
  const three = useRef<{ nowBot?: URDFRobot; planBot?: URDFRobot; tcpProbe?: URDFRobot; path?: THREE.Mesh; cup?: THREE.Mesh } | null>(null);
  const sceneParts = useRef<SceneParts | null>(null);
  const [showVoxels, toggleVoxels] = useToggle("hp.plan3d.octomap");
  const [showObjects, toggleObjects] = useToggle("hp.plan3d.objects");
  const [err, setErr] = useState<string | null>(null);
  const [ready, setReady] = useState(false);
  const [scrub, setScrub] = useState<number | null>(1); // default: hold the planned end pose; null = loop
  const tRef = useRef(0);

  useEffect(() => {
    const el = host.current!;
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    renderer.setClearColor(0x0d0f11);
    el.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 50);
    // three coords: x forward, y up, z = −ROS y. Look at the arm from behind-left, whole robot and table in frame.
    camera.position.set(-1.5, 1.9, 2.3);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0.55, 0.55, 0);
    controls.enableDamping = true;
    scene.add(new THREE.HemisphereLight(0xdfe6ee, 0x1a1d21, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.4);
    key.position.set(2, 4, 3);
    scene.add(key);
    // ROS is z-up; three is y-up
    const world = new THREE.Group();
    world.rotation.x = -Math.PI / 2;
    scene.add(world);
    const grid = new THREE.GridHelper(4, 16, C.gridMain, C.grid);
    grid.rotation.x = Math.PI / 2;
    world.add(grid);
    const tape = new THREE.Mesh(new THREE.PlaneGeometry(0.3, 0.03), new THREE.MeshBasicMaterial({ color: 0xf2b33d }));
    tape.position.set(0, 0, 0.001);
    world.add(tape);
    const cup = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.035, 0.1, 24), new THREE.MeshStandardMaterial({ color: C.cup }));
    cup.rotation.x = Math.PI / 2;
    cup.visible = false;
    world.add(cup);
    const path = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: C.path }));
    world.add(path);
    three.current = { path, cup };
    const sceneRoot = new THREE.Group();
    world.add(sceneRoot);
    sceneParts.current = { root: sceneRoot, attached: [] };
    Promise.all([loadRobot(C.now, 1), loadRobot(C.plan, 0.55), loadRobot(C.plan, 0)])
      .then(([n, p, probe]) => {
        world.add(n, p, probe);
        probe.visible = false;
        three.current = { ...three.current, nowBot: n, planBot: p, tcpProbe: probe };
        (window as unknown as { __plan3d: unknown }).__plan3d = three.current;
        setReady(true);
      })
      .catch((e) => setErr(String(e.message ?? e)));
    const resize = () => {
      const { clientWidth: w, clientHeight: h } = el;
      renderer.setSize(w, h, false);
      camera.aspect = w / Math.max(1, h);
      camera.updateProjectionMatrix();
    };
    const ro = new ResizeObserver(resize);
    ro.observe(el);
    resize();
    let raf = 0;
    let last = performance.now();
    const loop = (t: number) => {
      raf = requestAnimationFrame(loop);
      tRef.current += (t - last) / 1000;
      last = t;
      // meshes stream in after parse; keep each robot in its role colour
      const tc = three.current;
      if (tc?.nowBot) tint(tc.nowBot, tc.nowBot.userData.mat);
      if (tc?.planBot) tint(tc.planBot, tc.planBot.userData.mat);
      controls.update();
      renderer.render(scene, camera);
    };
    raf = requestAnimationFrame(loop);
    return () => {
      const sp = sceneParts.current;
      if (sp) {
        sp.attached.forEach(disposeTree);
        disposeTree(sp.root);
      }
      sceneParts.current = null;
      cancelAnimationFrame(raf);
      ro.disconnect();
      controls.dispose();
      renderer.dispose();
      el.removeChild(renderer.domElement);
    };
  }, []);

  // current robot: only from fresh joints
  useEffect(() => {
    const t = three.current;
    if (!t?.nowBot) return;
    t.nowBot.visible = !!now;
    setJoints(t.nowBot, now);
  }, [now, ready]);

  // plan: TCP path + target
  useEffect(() => {
    const t = three.current;
    if (!t?.path || !t.cup) return;
    if (!plan?.trajectory?.length) {
      t.path.visible = false;
      if (t.planBot) t.planBot.visible = false;
      t.cup.visible = false;
      return;
    }
    if (plan.target) {
      t.cup.visible = true;
      t.cup.position.set(plan.target[0], plan.target[1], plan.target[2]);
    } else t.cup.visible = false;
    if (t.tcpProbe) {
      const pts: THREE.Vector3[] = [];
      const tcp = t.tcpProbe.links["grasp_tcp"];
      for (const p of plan.trajectory) {
        setJoints(t.tcpProbe, p.joints);
        t.tcpProbe.updateMatrixWorld(true);
        if (tcp) {
          const w = new THREE.Vector3();
          tcp.getWorldPosition(w);
          pts.push(t.path.parent!.worldToLocal(w));
        }
      }
      t.path.geometry.dispose();
      const uniq = pts.filter((p, i) => i === 0 || p.distanceTo(pts[i - 1]) > 1e-4);
      t.path.geometry = uniq.length > 1 ? new THREE.TubeGeometry(new THREE.CatmullRomCurve3(uniq), 64, 0.012, 8) : new THREE.BufferGeometry();
      t.path.visible = uniq.length > 1;
    }
  }, [plan?.plan_id, ready]);

  // plan robot: loop through the trajectory, or hold the scrubbed point
  useEffect(() => {
    const id = window.setInterval(() => {
      const t = three.current;
      if (!t?.planBot || !plan?.trajectory?.length) return;
      t.planBot.visible = true;
      const traj = plan.trajectory;
      const T = traj[traj.length - 1].t || 1;
      const at = scrub != null ? scrub * T : (tRef.current % (T + 1.2)) - 0.6;
      const tt = Math.min(T, Math.max(0, at));
      let i = traj.findIndex((p) => p.t >= tt);
      if (i < 0) i = traj.length - 1;
      setJoints(t.planBot, traj[i].joints);
    }, 33);
    return () => window.clearInterval(id);
  }, [plan?.plan_id, scrub]);

  // planning scene: voxels are fetched separately, only when the octree changed (scene.octomap.seq)
  const voxSeq = scene?.octomap?.seq ?? 0;
  useEffect(() => {
    const sp = sceneParts.current;
    if (!sp || !showVoxels || sp.voxSeq === voxSeq) return;
    let cancelled = false;
    fetch("/api/scene/voxels", { cache: "no-store" })
      .then((r) => (r.ok ? (r.json() as Promise<SceneVoxels>) : Promise.reject(new Error(`HTTP ${r.status}`))))
      .then((v) => {
        if (cancelled || sceneParts.current !== sp) return;
        if (sp.voxels) disposeTree(sp.voxels);
        sp.voxels = buildVoxels(v);
        sp.voxels.visible = !!scene && !scene.stale;
        sp.voxSeq = v.seq;
        sp.root.add(sp.voxels);
      })
      .catch(() => undefined); // the next seq or toggle retries
    return () => {
      cancelled = true;
    };
  }, [voxSeq, showVoxels]);

  // world objects + collision point: rebuilt when their content changes
  const col = scene?.collision;
  const objKey = JSON.stringify([scene?.objects ?? [], col ? [col.p, col.a, col.b, col.frame, col.t_wall] : null]);
  useEffect(() => {
    const sp = sceneParts.current;
    if (!sp) return;
    if (sp.objects) disposeTree(sp.objects);
    const g = new THREE.Group();
    for (const o of scene?.objects ?? []) {
      const m = buildPrim(o, SC.object);
      if (m) g.add(m);
    }
    if (col && col.frame === "world") g.add(buildCollision(col.p, `${col.a} / ${col.b}`));
    sp.objects = g;
    sp.root.add(g);
  }, [objKey]);

  // attached objects ride on their link of the solid "now" robot (hidden with it when joints are stale)
  const attKey = JSON.stringify(scene?.attached ?? []);
  useEffect(() => {
    const sp = sceneParts.current;
    const bot = three.current?.nowBot;
    if (!sp) return;
    sp.attached.forEach(disposeTree);
    sp.attached = [];
    if (!bot) return;
    for (const a of scene?.attached ?? []) {
      const link = a.link ? bot.links[a.link] : undefined;
      const m = link ? buildPrim(a, SC.attached) : null;
      if (link && m) {
        link.add(m);
        sp.attached.push(m);
      }
    }
  }, [attKey, ready]);

  // visibility: the toggles, and a stale scene is never drawn as if it were current
  const fresh = !!scene && !scene.stale;
  useEffect(() => {
    const sp = sceneParts.current;
    if (!sp) return;
    if (sp.voxels) sp.voxels.visible = showVoxels && fresh && !!scene?.octomap;
    if (sp.objects) sp.objects.visible = showObjects && fresh;
    for (const a of sp.attached) a.visible = showObjects && fresh;
  });

  const oct = scene?.octomap;
  const sceneNote = !scene
    ? null
    : scene.stale
      ? `planning scene unknown${scene.error ? `: ${scene.error}` : scene.age != null ? ` (${fmtAge(scene.age)} old)` : ""}`
      : scene.error;
  const objNames = [...new Set((scene?.objects ?? []).map((o) => shortId(o.id) + (o.kind === "mesh" ? " (mesh, not drawn)" : "")))];
  const attNames = [...new Set((scene?.attached ?? []).map((o) => `${shortId(o.id)} on ${o.link}`))];

  return (
    <div className="plan3d">
      <div ref={host} style={{ position: "absolute", inset: 0 }} />
      {err && (
        <div className="empty3d">
          <b>Robot model unavailable</b>
          <span>{err}</span>
        </div>
      )}
      <div className="legend">
        <span>
          <i style={{ background: "#6b727c" }} />
          robot now {now ? "" : "· unknown (joints stale)"}
        </span>
        <span>
          <i style={{ background: "#f2b33d" }} />
          start spot (0, 0)
        </span>
        {plan ? (
          <>
            <span>
              <i style={{ background: "#6ea8fe" }} />
              plan #{plan.plan_id} · {plan.step} {scrub === 1 ? "· end pose" : ""} {planAge != null ? `· ${fmtAge(planAge)} old` : ""}
            </span>
            <span>
              <i style={{ background: "#6ea8fe", height: 2 }} />
              TCP path
            </span>
            {plan.target && (
              <span>
                <i style={{ background: "#ff5c4d" }} />
                detected target
              </span>
            )}
          </>
        ) : (
          <span className="muted">No plan yet this session</span>
        )}
        {scene && (
          <>
            {showVoxels && oct && fresh && (
              <span>
                <i style={{ background: HEIGHT_GRADIENT, height: 6 }} />
                OctoMap {oct.count.toLocaleString()} voxels · {Math.round(oct.resolution * 1000) / 10} cm{oct.truncated ? " · truncated" : ""}
              </span>
            )}
            {showObjects && fresh && objNames.length > 0 && (
              <span>
                <i style={{ background: "#5fd38d" }} />
                {objNames.join(", ")}
              </span>
            )}
            {showObjects && fresh && attNames.length > 0 && (
              <span>
                <i style={{ background: "#c58cff" }} />
                attached: {attNames.join(", ")}
                {now ? "" : " · hidden (joints stale)"}
              </span>
            )}
            {showObjects && col && (
              <span>
                <i style={{ background: "#ff3b30" }} />
                collision {col.a} / {col.b} · {fmtAge(col.age)} ago{col.frame !== "world" ? ` · in ${col.frame}, not drawn` : ""}
              </span>
            )}
            {sceneNote && <span className="muted">{sceneNote}</span>}
            {scene.skipped.length > 0 && <span className="muted">not drawn: {scene.skipped.join("; ")}</span>}
          </>
        )}
        <span className="scene-toggles">
          <button className={`btn sm ${showVoxels ? "on" : ""}`} aria-pressed={showVoxels} onClick={toggleVoxels} title="MoveIt OctoMap: occupied voxels">
            OctoMap
          </button>
          <button className={`btn sm ${showObjects ? "on" : ""}`} aria-pressed={showObjects} onClick={toggleObjects} title="MoveIt collision objects, attached objects and the last collision point">
            Objects
          </button>
        </span>
      </div>
      <div className="caveat">Joint sign/zero conventions not yet checked against RViz</div>
      {plan?.trajectory?.length ? (
        <div className="scrub">
          <button className="btn sm" onClick={() => setScrub(scrub == null ? 1 : null)}>
            {scrub == null ? "Hold" : "Loop"}
          </button>
          <input
            type="range"
            min={0}
            max={1}
            step={0.01}
            value={scrub ?? 0}
            onChange={(e) => setScrub(Number(e.target.value))}
            aria-label="Scrub planned trajectory"
          />
          <span>
            {plan.duration_sec.toFixed(1)} s · {plan.waypoints} wp
          </span>
        </div>
      ) : null}
    </div>
  );
}
