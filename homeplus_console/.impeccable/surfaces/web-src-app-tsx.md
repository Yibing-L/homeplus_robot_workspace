---
version: 1
slug: "web-src-app-tsx"
primary_target: "web/src/App.tsx"
related_targets: []
---

# Operator console (single page)

Mode: Operate. Audience: 1–2 ARL students beside the HomePlus robot, laptop on a cart, glancing at arm's length between physical tasks; most time is failure recovery. Task: know the true state, take the one next correct action, approve or reject motion with full knowledge (including the MoveIt plan in 3D), recover without touching the robot (Return to start, Undo, Set start here), STOP at any time. Constraints: docs/OPERATOR_CONSOLE_PLAN.md §3–§6 and §5.4 (derived state, three-valued truth, gated actions, STOP never covered, no "E-stopped" claim). Sim mode is signal-level Python, no ROS.

Direction history: Flowsheet (the roll's assignment) was built first. The user rejected it as too busy and light, and chose the Graphite challenger instead (2026-10-18).

## Direction contract

THESIS: A dark-first working surface where the robot's true state reads at a glance. Graphite panels are divided by one-pixel seams instead of cards or shadows, colour appears only where state lives, and STOP and motion actions sit apart in deliberate empty space. It refuses the cluttered light dashboard and the neon "cockpit" costume.

OWN-WORLD: Graphite ground #141619, panels #1B1E22 one step up, hairline seams #2A2E34, text #E6E8EB / #A3A9B1 / #737A84. State accents only: green #4CC38A, amber #F2B33D, red #FF5C4D, blue #6EA8FE (running/info). Unknown is a fine diagonal hatch at low contrast, never a colour. Geist for the UI, small caps for panel headers, Geist Mono for figures, topics and step names, tabular figures everywhere. No shadows, no gradients, no rounded cards: panels tile and persist.

STORY: The operator glances and sees the headline state (the most dangerous wins), whether every feed and component is live, and the single next action. Before approving they see what will move, as numbers and in 3D, against a server-side countdown. After a failure they recover from the console: undo, return to start, set start here.

FIRST VIEWPORT: Top row: an isolated STOP bay (264 px, red, Esc) separated by a seam from the headline (state in its accent colour, then sequence · step), with a row of quiet state pills under it (perception, motors, origin, link/control, SIM). Below, three tiled columns: health (7 rows with dot, rate, age, time strip), feeds (D405 overlay large, plus color, depth and D455, each with an age/fps caption; a Plan 3D tab that takes over during approvals), and the timeline (mono step names, notes right-aligned). Across the bottom is an action bar: approval, recovery, problem + fix, or preflight + start, with the motion button in its own bay on the right.

FORM: Graphite Console (dark-first developer console, catalog challenger digital-design-canon-dark-first-developer-console), chosen by the user over the assigned Flowsheet; seed:48f826d4a2221854. Signature move: the isolated motion bay. Every action that moves the robot lives in a separate right-hand bay behind a seam, outlined until armed and filled while held, so read-only and motion actions are never neighbours. Carried over: time strips under every signal (ticks per receipt; a silent signal grows a hatched gap) and hatching for unknown. Motion grammar: panels expand along one axis only, the hold-to-approve fill runs linearly for 1 s, and the countdown drains linearly.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance
