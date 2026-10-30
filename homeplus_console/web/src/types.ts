// Snapshot shape sent by the backend every 250 ms (homeplus_console/api.py).
// Every value that can go stale is nullable: null means unknown, never zero.

export type Severity = "ok" | "info" | "warn" | "bad" | "unknown";

export interface RobotHeadline {
  state: string;
  title: string;
  severity: Severity;
  sub: string | null;
  detail: string;
  since_wall: number | null;
}

export interface MotorPower {
  value: "verified" | "unverified" | "suspect" | "unknown";
  reasons: { kind: string; text: string }[];
  verified: { at: number; how: string } | null;
}

export interface Origin {
  status: "confirmed" | "unconfirmed" | "lost";
  at: number | null;
  by: string | null;
  session_id: string | null;
  note: string | null;
  lost_reason: string | null;
  lost_at: number | null;
  base_at_reset: { base_x: number; base_y: number; base_theta: number } | null;
}

export interface Declaration {
  at: number;
  by: string;
  note: string;
}

export interface Window {
  ok: boolean;
  consecutive: number;
  required: number;
  spread_cm: number | null;
  z_band: [number, number];
  z: number | null;
  position: number[] | null;
  two_candidates: boolean;
  samples: { x: number; y: number; z: number; age: number }[];
  reasons: string[];
  task_id: number;
}

export interface Perception {
  state: string;
  title: string;
  severity: Severity;
  detail: string;
  fix: string | null;
  task_id: number | null;
  requested_task: number;
  window: Window;
  diagnostics: {
    stage: string | null;
    reason: string | null;
    detail: string | null;
    frame_age_sec: number | null;
    cycle_sec: number | null;
    backend: string | null;
    model_text: string | null;
    age: number | null;
    fresh: boolean;
  };
}

export interface Feed {
  id: string;
  label: string;
  kind: "overlay" | "color" | "depth";
  topic: string | null;
  seq: number | null;
  age: number | null;
  fps: number;
  status: "live" | "slow" | "stale" | "frozen" | "off";
  frozen_sec: number;
  stale_after: number;
  strip: string;
  depth_range_m: [number, number] | null;
  note: string | null;
  source_age: number | null;
}

export interface Component {
  id: string;
  label: string;
  unit: string | null;
  unit_state: string | null;
  sub_state: string | null;
  pid: number | null;
  uptime_s: number | null;
  restarts: number | null;
  health: "ok" | "external" | "starting" | "silent" | "down" | "unknown";
  detail: string;
  value: string | null;
  observed: { age: number | null; fresh: boolean; rate_hz: number; strip: string; source: string };
  last_error: { t: number; text: string } | null;
  observed_only: boolean;
  external: string[];
  start_block: string | null;
}

export interface Step {
  name: string;
  type: string;
  status: "pending" | "running" | "done" | "failed" | "skipped" | "cancelled";
  t_start: number | null;
  t_end: number | null;
  note: string;
  requires_approval: boolean;
}

export interface LastResult {
  outcome: "succeeded" | "aborted" | "failed" | "cancelled";
  command: string | null;
  step: string | null;
  reason: string;
  t: number;
}

export interface Executor {
  fresh: boolean;
  age: number | null;
  structured: boolean;
  busy: boolean | null;
  command: string | null;
  request_token: string | null;
  sequence: {
    name: string;
    step_index: number;
    step_count: number;
    step_name: string;
    step_type: string;
    phase: string | null;
  } | null;
  steps: Step[];
  last_result: LastResult | null;
  last_rejected: { token: string; reason: string } | null;
  holding_cup: boolean | null;
  undo_stack: UndoRecord[];
  capabilities: { undo: boolean; resume: boolean; return_to_start: boolean };
  resumable: { command: string; step_index: number; step: string; step_count: number } | null;
}

export interface UndoRecord {
  undo_id: number;
  step: string;
  type: string;
  command: string;
  traj_file: string | null;
  start_joints: Record<string, number>;
  end_joints: Record<string, number>;
  gripper_before: number;
  t_done: number;
  endpoint_verified: boolean;
  reversible: boolean;
}

export interface Recovery {
  pose: { x: number; y: number; theta: number };
  home_distance_m: number;
  home_bearing_deg: number;
  home_turn_deg: number;
  undo_count: number;
  undo_next: UndoRecord | null;
}

export interface PlanPreview {
  plan_id: number;
  step: string;
  requires_approval: boolean;
  joints: { name: string; start: number; end: number; max_excursion: number }[];
  base_delta: { dx: number; dy: number; dtheta_deg: number };
  duration_sec: number;
  waypoints: number;
  tcp_end: number[];
  target: number[] | null;
  target_distance_m: number | null;
  target_age_sec: number | null;
  command: string | null;
  undo: { method: "reverse_replay" | "fallback_plan" | "restore_gripper"; reason: string; traj_file: string | null; of_step: string } | null;
  base_start: { x: number; y: number; theta: number };
  base_end: { x: number; y: number; theta: number };
  trajectory: { t: number; joints: Record<string, number> }[];
}

/** MoveIt planning scene (scene.py), polled ~1 Hz from /get_planning_scene. Poses are {p: [x,y,z], q: [x,y,z,w]}. */
export interface ScenePrim {
  id: string;
  kind: "box" | "sphere" | "cylinder" | "cone" | "mesh" | string;
  dims: number[]; // SolidPrimitive dimensions: box [x,y,z], sphere [r], cylinder/cone [height, radius]
  p: number[];
  q: number[];
  link?: string; // attached objects: pose is relative to this robot link
}

export interface SceneView {
  polling: boolean;
  age: number | null;
  stale: boolean;
  error: string | null;
  frame: string | null;
  octomap: {
    seq: number;
    id: string;
    format: "full" | "binary";
    bytes: number;
    resolution: number;
    count: number;
    truncated: boolean;
    decode_ms: number | null;
    error: string | null;
  } | null;
  objects: ScenePrim[]; // world frame
  attached: ScenePrim[];
  skipped: string[];
  collision: { a: string; b: string; frame: string; p: number[]; t_wall: number; age: number } | null;
}

/** GET /api/scene/voxels: base64 little-endian int16 rows (x, y, z, level); centre = (xyz + 2^level/2) * resolution. */
export interface SceneVoxels {
  seq: number;
  count: number;
  resolution: number | null;
  origin: { p: number[]; q: number[] };
  data: string;
}

export interface Approval {
  plan_id: number | null;
  step: string;
  deadline_wall: number | null;
  remaining_sec: number | null;
  timeout_sec: number;
  state_age: number | null;
  preview: PlanPreview | null;
  preview_ready: boolean;
}

export interface NextAction {
  id: string;
  severity: Severity;
  problem: string;
  detail: string;
  fix: string;
  action: { name: string; args?: Record<string, unknown>; label: string } | null;
  flow: "set_start" | null;
}

export interface CheckItem {
  id: string;
  label: string;
  ok: boolean;
  detail?: string;
  panel?: string | null;
  declared?: { at: number; by: string } | null;
  auto?: boolean;
}

export interface FlowStep {
  id: string;
  title: string;
  status: "done" | "active" | "pending" | "error";
  note: Record<string, unknown> | null;
}

export interface Flow {
  id: string;
  kind: "set_start";
  method: "zero" | "reset";
  push: boolean;
  options: { label: string; choice: string; primary?: boolean }[];
  reason: string;
  status: "active" | "error" | "done" | "cancelled";
  step: string;
  step_index: number;
  steps: FlowStep[];
  waiting_for: string;
  can_confirm: boolean;
  confirm_label: string | null;
  error: string | null;
  crosscheck: "consistent" | "contradicted" | null;
  step_age: number;
}

export interface TrackedAction {
  request_id: string;
  action: string;
  args: Record<string, unknown>;
  status: "sent" | "accepted" | "confirmed" | "not_observed" | "not_started" | "rejected" | "failed";
  detail: string;
  t_sent: number;
}

export interface StopResult {
  t: number;
  status: "sent" | "confirmed" | "not_confirmed";
  detail: string;
  request_id: string;
}

export interface LogLine {
  seq: number;
  t_wall: number;
  node: string;
  level: string;
  text: string;
  important: boolean;
  source: string;
}

export interface Fault {
  id: number;
  code: string;
  text: string;
  onset: number;
  acknowledged: boolean;
  hint: string;
}

export interface Allowed {
  allowed: boolean;
  reason: string | null;
  motion: boolean;
  // start_task only: per-task gate on the gripper's load (actions.HOLDING_TASKS), keyed by task id
  tasks?: Record<string, { allowed: boolean; reason: string | null }>;
}

export interface Snapshot {
  seq: number;
  server_wall: number;
  mode: "sim" | "ros";
  profile: string;
  degraded: string | null;
  robot: RobotHeadline;
  motor_power: MotorPower;
  origin: Origin;
  estop_declaration: Declaration | null;
  perception: Perception;
  feeds: Feed[];
  components: Component[];
  joints: {
    fresh: boolean;
    age: number | null;
    values: Record<string, number> | null;
    last_values: Record<string, number> | null;
    ready: boolean | null;
    arm_still: boolean | null;
    session_id: string | null;
    si: Record<string, number> | null;
  };
  recovery: Recovery | null;
  plan_preview: PlanPreview | null;
  plan_preview_age: number | null;
  executor: Executor;
  approval: Approval | null;
  faults: Fault[];
  preflight: CheckItem[];
  declared: CheckItem[];
  next_actions: NextAction[];
  flow: Flow | null;
  stop: StopResult | null;
  tracked_actions: TrackedAction[];
  logs: LogLine[];
  units: string[];
  allowed: Record<string, Allowed>;
  control: { holder: string | null; held: boolean; you: boolean };
  sim: Record<string, unknown> | null;
  scene?: SceneView;
}
