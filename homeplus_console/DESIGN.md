---
name: HomePlus Console
description: Dark-first operator console for the HomePlus robot; colour only where state lives.
colors:
  ground: "#121417"
  panel: "#1a1d21"
  panel-2: "#20242a"
  raise: "#262b31"
  field: "#0d0f11"
  seam: "#2a2e34"
  seam-2: "#353a42"
  ink: "#e6e8eb"
  ink-2: "#a3a9b1"
  ink-3: "#7b828c"
  ok: "#4cc38a"
  ok-dim: "#4cc38a24"
  warn: "#f2b33d"
  warn-dim: "#f2b33d1f"
  bad: "#ff5c4d"
  bad-dim: "#ff5c4d1f"
  info: "#6ea8fe"
  info-dim: "#6ea8fe1c"
  stop: "#e5483a"
  stop-hover: "#f05a4c"
typography:
  state-headline:
    fontFamily: "Geist Variable, Geist, system-ui, sans-serif"
    fontSize: "28px"
    fontWeight: 700
    lineHeight: 1.4
    letterSpacing: "-0.01em"
  band-title:
    fontFamily: "Geist Variable, Geist, system-ui, sans-serif"
    fontSize: "21px"
    fontWeight: 650
    lineHeight: 1.2
    letterSpacing: "-0.01em"
  body:
    fontFamily: "Geist Variable, Geist, system-ui, sans-serif"
    fontSize: "15px"
    fontWeight: 400
    lineHeight: 1.4
    fontFeature: "tnum"
  meta:
    fontFamily: "Geist Variable, Geist, system-ui, sans-serif"
    fontSize: "13px"
    fontWeight: 400
    lineHeight: 1.4
  label:
    fontFamily: "Geist Variable, Geist, system-ui, sans-serif"
    fontSize: "11.5px"
    fontWeight: 650
    letterSpacing: "0.08em"
  mono:
    fontFamily: "Geist Mono Variable, Geist Mono, ui-monospace, monospace"
    fontSize: "13px"
    fontWeight: 400
    fontFeature: "tnum"
  countdown:
    fontFamily: "Geist Mono Variable, Geist Mono, ui-monospace, monospace"
    fontSize: "56px"
    fontWeight: 500
    lineHeight: 1
    letterSpacing: "-0.02em"
rounded:
  none: "0px"
  xs: "3px"
  sm: "4px"
  md: "5px"
  lg: "6px"
spacing:
  seam: "1px"
  xs: "4px"
  sm: "8px"
  md: "12px"
  lg: "16px"
  xl: "20px"
  xxl: "24px"
  row: "42px"
  panel-head: "36px"
components:
  button:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink}"
    rounded: "{rounded.md}"
    padding: "0 14px"
    height: "38px"
  button-hover:
    backgroundColor: "{colors.raise}"
  button-primary:
    backgroundColor: "{colors.ink}"
    textColor: "{colors.ground}"
    rounded: "{rounded.md}"
    padding: "0 14px"
    height: "38px"
  button-motion:
    backgroundColor: "{colors.ok-dim}"
    textColor: "{colors.ok}"
    rounded: "{rounded.md}"
    padding: "0 14px"
    height: "50px"
  button-motion-disabled:
    backgroundColor: "transparent"
    textColor: "{colors.ink-3}"
  stop:
    backgroundColor: "{colors.stop}"
    textColor: "#ffffff"
    rounded: "{rounded.lg}"
    height: "80px"
    width: "220px"
  stop-hover:
    backgroundColor: "{colors.stop-hover}"
  chip:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink-2}"
    rounded: "{rounded.md}"
    padding: "0 11px"
    height: "30px"
  chip-bad:
    backgroundColor: "{colors.bad-dim}"
    textColor: "{colors.bad}"
  panel:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.ink}"
    rounded: "{rounded.none}"
  panel-head:
    textColor: "{colors.ink-2}"
    typography: "{typography.label}"
    padding: "0 16px"
    height: "36px"
  motion-bay:
    backgroundColor: "{colors.field}"
    rounded: "{rounded.none}"
    padding: "16px 20px"
---

# Design System: HomePlus Console

## Overview

**Creative North Star: "Graphite Console"**

A dark-first working surface where the robot's true state reads at arm's length. Graphite panels tile edge to edge and persist; they are divided by one-pixel seams, not cards or shadows. Colour is reserved for state, so a calm screen is grey and any colour on it means something. STOP and every action that moves the robot sit apart in their own bays behind seams, against the darkest ground.

The density is a working console's: 42px rows, 36px panel headers, tabular figures everywhere. Unknown is never a colour: it is a fine diagonal hatch. The system rejects the cluttered light dashboard and the neon "cockpit" costume.

**Key Characteristics:**
- One-pixel seam tiling: the grid's gap shows the seam colour through.
- Four state accents (ok, warn, bad, info), each paired with a shape and a word.
- Hatch for unknown, stale, silent and link-lost.
- An isolated STOP bay at top left and an isolated motion bay at bottom right.
- Geist for prose, Geist Mono for ids, figures, topics and step names.

## Colors

Near-black graphite neutrals in four lifted steps, with saturated accents that appear only where state lives.

### Primary
- **Signal Green** (ok): live, verified, done, and the motion action colour (outlined motion buttons, hold-to-approve fill).
- **Caution Amber** (warn): awaiting approval, silent signals, declarations, SIM, the "Causes motion" bay label and the countdown.
- **Fault Red** (bad): down, failed, aborted, contradicted, stale veils, the link-lost overlay border.
- **Running Blue** (info): running steps, starting signals, planned paths, focus rings and links.

Each accent has a translucent `-dim` partner (12 to 14% alpha) used only as a row or cell wash behind that state.

### Secondary
- **STOP Vermilion** (stop / stop-hover): STOP only. It is deliberately a separate token from Fault Red so the button never reads as a status.

### Neutral
- **Graphite Ground** (ground): the page.
- **Graphite Panel** (panel): every tiled panel, header and band cell.
- **Lifted Graphite** (panel-2): chips, button rest, row hover.
- **Raised Graphite** (raise): button hover, transient result notes, the link-lost card.
- **Well Black** (field): camera and 3D wells, the STOP bay and the motion bay; the darkest surface marks things that are not read-only panels.
- **Seam** (seam): one-pixel dividers and row rules. **Seam Strong** (seam-2): control borders, the neutral band rule, inactive tracks.
- **Ink** (ink), **Ink Secondary** (ink-2), **Ink Tertiary** (ink-3): primary text, supporting text, labels and meta.

### Named Rules
**The State-Only Colour Rule.** Accents appear only to express state or the motion affordance. Chrome, headers and idle controls stay graphite.

**The Hatch-Is-Unknown Rule.** Unknown, stale, silent and link-lost are a 135° hatch (`1px` ink-2 line at 12% every 6px; dense variant 20% every 4px), never a colour.

**The Never-Colour-Alone Rule.** Every state carries a shape (see State marks) and a word.

## Typography

**Body Font:** Geist Variable (with system-ui)
**Label/Mono Font:** Geist Mono Variable (with ui-monospace)

**Character:** A neutral grotesk for prose and decisions, a mono for anything the operator might compare against a terminal: step names, topics, node ids, figures, ages, rates.

### Hierarchy
- **State headline** (700, 28px; 24px below 1360px wide, uppercase, -0.01em): the robot state, coloured by its accent.
- **Band title** (650, 21px, 1.2): the one problem or decision in the action bar. A mono variant (500, 20px) is used when the title is a step name.
- **Body** (400, 15px, 1.4): default. The fix sentence runs at 16.5px, max 70ch.
- **Meta** (400, 13px, ink-3): the line *after* a heading that carries plan id, step, time.
- **Label** (600 to 650, 11 to 12px, 0.07 to 0.08em, uppercase): panel headers, tabs, table row keys, feed tags.
- **Mono** (13px; 11.5 to 14.5px by context): ids, topics, step names, figures, log lines.
- **Countdown** (mono 500, 56px): approval seconds left.

### Named Rules
**The Mono-Is-Machine Rule.** Anything copied from or comparable to the robot (topic, node, step, plan id, numbers with units) is Geist Mono. Prose is never mono.

**The Tabular Rule.** `font-variant-numeric: tabular-nums` is set on body; figures never jitter as they update.

**The Meta-After Rule.** Headings stand first; context follows on a meta line beneath. No label line sits above a heading.

## Layout

A full-viewport grid (`min-width: 1180px`) of rows: header, main, action bar, footer. Every grid sets `gap: 1px` on a seam-coloured background, so the seams are the gaps.

- **Header** (min 112px): STOP bay in the side column (`clamp(280px, 22vw, 340px)`; 268px below 1360px) | headline, then a row of state pills.
- **Main**: three columns, side | flexible | right (`clamp(300px, 25vw, 400px)`; 300px below 1360px): health, feeds / Plan 3D, timeline.
- **Action bar** (min 190px; 176px below 820px tall): the same three columns: situation | detail | motion bay. A 2px top rule takes the band's state colour.
- **Footer** (min 54px): read-only and setup controls, the log line, Logs.

Panel interiors pad 16px horizontally; band cells pad 16px 20px. Rows are 42px (36px below 820px tall). Drawers slide in from the right below the header (`top: 113px`), so STOP is never covered.

## Elevation & Depth

Flat. Depth is tonal (field below ground below panel below panel-2 below raise) and separated by seams. Only `box-shadow` uses are inset 1 to 2px rings that draw shapes: the STOP inner hairline and the hollow state marks. Overlays dim with a translucent black scrim (`#0008`) and never cover the header.

### Named Rules
**The Seam-Not-Shadow Rule.** Panels never cast shadows or float; separation is a 1px seam or a tonal step.

## Shapes

Panels, bays and wells are square (0px) and tile edge to edge. Controls get small radii: 3px for tags, kbd and veil labels; 4px for in-panel overlays, fields and choice tiles; 5px for buttons and chips; 6px for STOP and the link-lost card. Progress tracks are 4px tall with 2px ends.

## Components

### Buttons
- **Shape:** gently squared (5px), 38px tall; sm 30px, lg 50px.
- **Default:** panel-2 fill, seam-2 border, ink text, 550 weight, 14px. Hover lifts to raise with an ink-3 border.
- **Primary:** inverted ink fill with ground text; used for the selected option in a toggle group.
- **Motion:** outlined Signal Green on an ok-dim wash, always with the move icon. Disabled drops to transparent with seam border and ink-3 text.
- **Focus:** 2px Running Blue outline, 2px offset.
- **Icons:** Lucide line icons at 15 to 20px.

### STOP
Vermilion, 80px tall, max 220px wide, 32px 700 uppercase with 0.06em tracking, an inset hairline, and an `Esc` kbd beneath. It lives alone in a field-coloured bay at top left. It never asks for confirmation. Hover brightens; active nudges 1px down.

### Motion Bay
The action bar's right cell, on field, behind a seam. Every action that moves the robot (start, hold-to-approve, reject, undo, return to start, retry, gripper) lives here and is confirmed inline in the bay, never in a modal, so the feeds and health stay visible. A small label heads the bay: "Causes motion" in amber, or "Controls" in ink-3. **Hold to approve** is 64px tall at 18px; a green fill (30% opacity) runs linearly for 1000ms while held.

### State marks
8px marks where shape carries the meaning and colour reinforces it: **filled dot** = ok / running; **hollow ring** (2px inset) = warn, silent, starting, declared; **red bar** (9x3px, 1px radius) = down / bad; **hatched square** (dense hatch, 1px ink-3 ring) = unknown.

### State pills (chips)
30px, 5px radius, panel-2, seam-2 border: an uppercase label in ink, then a value in ink-2. Variants: warn (amber value, hollow ring), bad (bad-dim wash, red border at 50%), unknown (hatched fill), declared (dashed amber border), SIM (amber border and text).

### Panel header
36px, a seam underneath, label typography in ink-2 on the left, a quiet ink-3 status on the right. Embedded ids switch to mono without case or tracking.

### Health row and time strip
A 4-column grid row: mark | name over mono source | rate | time strip. The strip is a 16px SVG with one ink-3 tick per receipt, seam gridlines, an ink-2 "now" line and an amber onset line; a silent signal grows a hatched gap. Silent and down rows take the hatch wash and colour the source line.

### Feed well, stale and frozen veils
Feeds sit in field wells with an uppercase tag top left and a 28px caption (mono age, fps, strip). Stale, frozen or off feeds turn the image grayscale at 45% brightness under a dense hatch veil, with a centred bordered word (STALE / FROZEN in red) and a mono reason. The age turns red.

### Link-lost overlay
When the snapshot link is lost, main and the action bar go grayscale (60% brightness), become inert and take a dense hatch overlay. Ages and rates are struck through in ink-3. A centred raise card with a red border states the loss.

### Countdown
Mono 56px amber seconds over a 4px linear drain (250ms linear steps); both turn red when low.

## Do's and Don'ts

### Do:
- **Do** tile panels with `gap: 1px` over the seam colour; keep panels square and persistent.
- **Do** pair every state with a mark shape and a word, and use the hatch for anything unknown or stale.
- **Do** put every robot-moving action in the motion bay, outlined green with the move icon, confirmed inline.
- **Do** set ids, topics, step names and figures in Geist Mono with tabular numerals.
- **Do** follow a heading with its meta line; label panels with 36px uppercase-tracked headers.
- **Do** keep STOP alone in its bay, uncovered by drawers or overlays.

### Don't:
- **Don't** use accent colour for chrome, decoration or idle controls.
- **Don't** add drop shadows, floating cards or rounded panels. The hatch and data colour-scales are the only gradients.
- **Don't** confirm motion in a modal, or place a motion action beside read-only controls.
- **Don't** put a label line (kicker) above a heading.
- **Don't** show a last-known value as if live; veil it or hatch it.
