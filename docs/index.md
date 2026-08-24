# Robo-Tech Micromouse — Documentation

## The Problem

A micromouse must autonomously solve a 16×16 maze whose walls are unknown beforehand: start in a corner, reach the center 2×2 goal while mapping walls, then return to start and sprint the shortest known path — within a time limit, no external aid. That demands four concurrent capabilities: **sense** (three-sided wall detection, heading, odometry), **decide** (revise the route as walls are discovered), **act** (precise cell-to-cell motion and 90° turns), and **remember** (survive a mid-run reset by persisting the map).

This firmware runs that mission on an ESP32 with a BNO08x attitude sensor, three VL53L0X rangefinders, dual encoder motors, and NVS-backed maze storage. Today the exploration phase works (defects tracked below); the speed-run phase is not implemented; and switching simulator ↔ hardware still requires editing code.

## The Solution

A layered firmware with two strategy seams — infrastructure (sensors, actuators, simulator) behind thin interfaces, a hardware-free domain core (`MazeMap`/`Pose`/distances), an application `Navigator` orchestrating sense → solve → move, and both the maze solver (A* today) and the environment (sim or real) selected in one place. The full story lives in [Overview](overview/index.md); the work to get there is tracked as Design Evolution items RD-35–RD-40.

## Map

| Section | Answers |
|---|---|
| [Overview](overview/index.md) | The problem (example maze drawn whole, start/goal, mission phases) and the solution shape — start here |
| [Architecture](architecture/index.md) | What exists today: layers, pin map, sensors, motors, API surface |
| [Design Decisions](design-decisions/index.md) | Why it is shaped that way |
| [Algorithms](algorithms/index.md) | How solving and motion control work — including the example maze explored and solved |
| [R&D Items](rd-items/index.md) | Tracked defects *and* design evolution |

## R&D Backlog at a Glance

**40 items** across six areas. Full table on the [R&D Items hub](rd-items/index.md).

| Priority | Meaning | Count |
|---|---|---|
| P0 | Critical — blocks correct operation | 1 |
| P1 | High — broken feature or hardware risk | 8 |
| P2 | Medium — degraded behavior / friction / unblocks testing | 14 |
| P3 | Low — quality & maintainability | 17 |

**Fix first:** [RD-01 Heading correction disabled](rd-items/control/rd-01-heading-correction-disabled.md) (P0) · [RD-05 Solver can leave the maze](rd-items/navigation/rd-05-fallback-move-north-oob.md) · [RD-06 Per-cell flash writes](rd-items/reliability/rd-06-flash-wear-per-cell-save.md).
**Build toward:** [RD-35 Hardware abstraction](rd-items/design-evolution/rd-35-hardware-abstraction-layer.md) and [RD-36 Environment seam](rd-items/design-evolution/rd-36-environment-seam-sim-real.md) unlock simulation, host-side testing, and everything after them.

---

*Generated from the automated deep code review of 2026-08-24; design-evolution items added from the architecture recommendations review.*
