# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Stack

Set by the implementation plan (`~/Documents/arl/parallel_prompts/UI_PLAN.md` §8.2): Python 3.10 backend (rclpy + FastAPI/uvicorn + WebSocket) serving a static React 18 + TypeScript + Vite frontend, `zustand` for the snapshot store, plain CSS, no component framework. Source in `homeplus_console/web/`, build output in `homeplus_console/homeplus_console/static/`. Node is needed only to build.

## Users

One or two ARL students operating the HomePlus mobile manipulator in the lab. They stand next to the robot with a laptop on a cart and look at the console between physical tasks: pressing the E-stop, pushing the robot to the start spot, placing a cup. They read it at arm's length and often use it one-handed on a trackpad. Sessions are long and stressful, and most of the time goes to recovering from failures.

## Product Purpose

Replace the 5–7 terminal workflow (`HOMEPLUS_CHEATSHEET.md`) with one browser console that never shows a state that isn't true and makes it hard to run the robot in a bad state. Success means a full cup pickup run from the console alone, where the operator always knows the true state and the next correct action.

## Positioning

The browser never holds its own state. Every displayed fact comes from the backend's snapshot, carries its source and age, and is three-valued: true, false or unknown. When data is stale, the console says UNKNOWN instead of showing the last value.

## Operating Context

- Primary display: a laptop in Chrome, 1440–1920 px wide. It must also work on a 13" MacBook (1440×900, about 1280 px usable).
- Two deployments: Yibing's Ubuntu laptop (everything local), and a MacBook with ROS in a UTM VM, the D405 streamed from macOS, and MolmoPoint on an AWS GPU.
- Recurring failures the console must surface: E-stop or power off (the arm silently stays folded, or the firmware reports TRACKING_ERROR or SETTLE_TIMEOUT); an Arduino reset zeroes base odometry wherever the robot is; Arduino USB drop-outs; the camera sender dying silently while the detector keeps using minutes-old frames; garbage or alternating detections; 60 s approval timeouts; terminals that were "cancelled" but still executed.
- The pick_up_coffee sequence has 9 steps. Some steps move the robot without approval.

## Capabilities and Constraints

- **The console cannot observe the E-stop.** It never says "E-stopped" or "E-stop released". It shows Motor power as VERIFIED, UNVERIFIED or SUSPECT, inferred from motion. Operator declarations (for example "E-stop pressed") are shown as declarations with who and when, and labelled unverified.
- STOP is always available, from any client and in any state, and never asks for confirmation. It is bound to Esc.
- The server gates and audits every action. Motion actions need confirmation. Approval is hold-to-approve, scoped to a plan id, and its countdown comes from the executor's deadline.
- One controlling browser at a time holds a control lease; any other browser is view-only plus STOP.
- Sim mode (this phase): a signal-level Python sim feeds the backend's SignalStore without ROS, so the console can be developed on a Mac. The ROS IO layer and the plan's ROS fake nodes come with M0.
- Robot code is not modified in this phase. The sim emits the §7.1 `executor_state` and §7.2 `plan_preview` schemas.

## Brand Commitments

None beyond the lab tool's name, "HomePlus". It is an internal research tool, not a product with a brand.

## Evidence on Hand

Real topic names, log strings, step names and failure modes are in the plan and the robot code. There is no real camera footage in the repo. Sim video frames are synthetic and must be labelled SIM.

## Product Principles

1. Truth over reassurance: show unknown instead of stale, and show a declaration instead of a guess.
2. The most dangerous condition wins the headline.
3. Every problem comes with its one next action.
4. Motion is never ambiguous: what will move, by how much, and who allowed it.
5. Calm under failure: most of a session is recovery, so the console must not shout constantly.

## Accessibility & Inclusion

Readable at arm's length. State is never conveyed by colour alone; it always has a word and a shape. Every action is reachable by keyboard and works with one hand on a trackpad.
