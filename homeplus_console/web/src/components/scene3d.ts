import * as THREE from "three";
import type { ScenePrim, SceneVoxels } from "../types";

// MoveIt's planning scene in the Plan3D world group (ROS world frame, z up):
//   OctoMap occupied voxels = one InstancedMesh of translucent cubes coloured by height
//   collision objects        = translucent primitives with edges (world objects green, attached violet)
//   collision point          = red sphere + label from the executor's "Collision overlay" log line

export const SC = { object: 0x5fd38d, attached: 0xc58cff, collision: 0xff3b30 };
const Z_LO = 0.0; // height colour ramp (m, world z): blue low → red high, as RViz colours an OctoMap
const Z_HI = 1.6;

export function heightColor(z: number, out = new THREE.Color()): THREE.Color {
  const v = Math.min(1, Math.max(0, (z - Z_LO) / (Z_HI - Z_LO)));
  return out.setHSL((1 - v) * 0.66, 0.85, 0.55);
}

export const HEIGHT_GRADIENT = `linear-gradient(90deg, ${[0, 0.25, 0.5, 0.75, 1]
  .map((v) => "#" + heightColor(Z_LO + v * (Z_HI - Z_LO)).getHexString())
  .join(", ")})`;

function decodeRows(b64: string): Int16Array {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Int16Array(bytes.buffer, 0, bytes.length >> 1); // little-endian rows; browsers are LE
}

/** Voxels in their octree frame; the caller's group carries the OctoMap origin pose. */
export function buildVoxels(v: SceneVoxels): THREE.Group {
  const g = new THREE.Group();
  g.position.set(v.origin.p[0], v.origin.p[1], v.origin.p[2]);
  g.quaternion.set(v.origin.q[0], v.origin.q[1], v.origin.q[2], v.origin.q[3]);
  if (!v.count || !v.resolution) return g;
  const rows = decodeRows(v.data);
  const n = Math.min(v.count, rows.length >> 2);
  const geom = new THREE.BoxGeometry(1, 1, 1);
  const mat = new THREE.MeshStandardMaterial({ transparent: true, opacity: 0.5, depthWrite: false, roughness: 0.9 });
  const mesh = new THREE.InstancedMesh(geom, mat, n);
  const m = new THREE.Matrix4();
  const pos = new THREE.Vector3();
  const scl = new THREE.Vector3();
  const one = new THREE.Quaternion();
  const world = new THREE.Vector3();
  const c = new THREE.Color();
  const r = v.resolution;
  for (let i = 0; i < n; i++) {
    const k = i * 4;
    const span = 2 ** rows[k + 3];
    const size = span * r;
    pos.set((rows[k] + span / 2) * r, (rows[k + 1] + span / 2) * r, (rows[k + 2] + span / 2) * r);
    scl.setScalar(size * 0.92); // a small gap keeps neighbouring cubes readable
    m.compose(pos, one, scl);
    mesh.setMatrixAt(i, m);
    world.copy(pos).applyQuaternion(g.quaternion).add(g.position);
    mesh.setColorAt(i, heightColor(world.z, c));
  }
  mesh.instanceMatrix.needsUpdate = true;
  if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  mesh.frustumCulled = false; // instance bounds aren't the unit cube's
  g.add(mesh);
  return g;
}

function primGeometry(p: ScenePrim): THREE.BufferGeometry | null {
  const d = p.dims;
  switch (p.kind) {
    case "box":
      return new THREE.BoxGeometry(d[0], d[1], d[2]);
    case "sphere":
      return new THREE.SphereGeometry(d[0], 20, 14);
    case "cylinder": // SolidPrimitive: [height, radius], axis = z; three's cylinder axis is y
      return new THREE.CylinderGeometry(d[1], d[1], d[0], 24).rotateX(Math.PI / 2);
    case "cone":
      return new THREE.ConeGeometry(d[1], d[0], 24).rotateX(Math.PI / 2);
    default:
      return null; // meshes are listed in the legend, not drawn
  }
}

/** One primitive in its pose (world frame, or relative to its link for attached objects). */
export function buildPrim(p: ScenePrim, color: number): THREE.Object3D | null {
  const geom = primGeometry(p);
  if (!geom) return null;
  const mat = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.32, depthWrite: false, roughness: 0.8 });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25), new THREE.LineBasicMaterial({ color })));
  mesh.position.set(p.p[0], p.p[1], p.p[2]);
  mesh.quaternion.set(p.q[0], p.q[1], p.q[2], p.q[3]);
  mesh.userData.sceneId = p.id;
  return mesh;
}

function textSprite(text: string): THREE.Sprite {
  const cv = document.createElement("canvas");
  const ctx = cv.getContext("2d")!;
  const font = "600 28px Geist Variable, system-ui, sans-serif";
  ctx.font = font;
  cv.width = Math.ceil(ctx.measureText(text).width) + 24;
  cv.height = 44;
  ctx.font = font;
  ctx.fillStyle = "#0d0f11d0";
  ctx.fillRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = "#ffd60a";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 12, cv.height / 2);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  const h = 0.035;
  sp.scale.set((h * cv.width) / cv.height, h, 1);
  sp.center.set(0.5, 0);
  sp.renderOrder = 10;
  return sp;
}

export function buildCollision(p: number[], label: string): THREE.Group {
  const g = new THREE.Group();
  g.position.set(p[0], p[1], p[2]);
  const dot = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), new THREE.MeshBasicMaterial({ color: SC.collision, depthTest: false }));
  dot.renderOrder = 9;
  g.add(dot);
  const sp = textSprite(label);
  sp.position.set(0, 0, 0.025);
  g.add(sp);
  return g;
}

export function disposeTree(o: THREE.Object3D) {
  o.traverse((c) => {
    const m = c as THREE.Mesh;
    m.geometry?.dispose();
    const mats = Array.isArray(m.material) ? m.material : m.material ? [m.material] : [];
    for (const mat of mats) {
      (mat as THREE.SpriteMaterial).map?.dispose();
      mat.dispose();
    }
  });
  o.removeFromParent();
}
