[Home](../index.md) › **R&D Items**

# R&D Items

All findings from the August 2026 deep code review plus the architecture recommendations, reframed as actionable R&D items. Every item has its own page with problem statement, evidence, proposed approach, and acceptance criteria.

## Backlog by Subsystem

| Section | Items | Highest priority | Focus |
|---|---|---|---|
| [Control & Motion](control/index.md) | 6 | P0 | Heading hold, turn settling, task scheduling, tuning |
| [Sensing](sensing/index.md) | 6 | P1 | IMU refresh + calibration, ToF validity windows |
| [Navigation](navigation/index.md) | 7 | P1 | Solver safety, return trip, direction state |
| [Reliability](reliability/index.md) | 3 | P1 | Flash wear, init hangs, switch polarity |
| [Code Health](code-health/index.md) | 12 | P2 | Dead code, API shape, magic numbers |
| [Design Evolution](design-evolution/index.md) | 6 | P1 | Layering (HAL + application services), sim/real seam, pluggable solver, domain naming |

The Design Evolution track is different in kind: not defects but structural improvements that unblock simulation, host-side testing, algorithm experimentation, and eventually the speed-run phase.

## Priority Distribution

| Priority | Meaning | Count |
|---|---|---|
| P0 | Critical — blocks correct operation | 1 |
| P1 | High — broken feature or hardware risk | 8 |
| P2 | Medium — degraded behavior / latent risk | 14 |
| P3 | Low — quality & maintainability | 17 |

## Suggested Attack Order

**Defect track** (correctness first):

1. [RD-01](control/rd-01-heading-correction-disabled.md) — heading correction ×0 (P0): one-line fix, huge straight-line payoff.
2. [RD-05](navigation/rd-05-fallback-move-north-oob.md) + [RD-17](navigation/rd-17-unbounded-search-loop.md) — make the solver unable to leave the maze or spin forever.
3. [RD-03](sensing/rd-03-imu-background-refresh-skipped.md) + [RD-04](sensing/rd-04-imu-calibration-window-broken.md) — make background sensing real so calibration works.
4. [RD-06](reliability/rd-06-flash-wear-per-cell-save.md) — stop writing flash every cell before practice accumulates wear.
5. Then Control and Sensing P2s, sweeping Code Health opportunistically.

**Design evolution track** (run in parallel once the P0/P1 defects are stable):
[RD-35](design-evolution/rd-35-hardware-abstraction-layer.md) → [RD-36](design-evolution/rd-36-environment-seam-sim-real.md) → [RD-37](design-evolution/rd-37-application-services-navigator.md) → [RD-38](design-evolution/rd-38-solver-strategy-seam.md) → [RD-39](design-evolution/rd-39-domain-rename-taxonomy.md) → [RD-40](design-evolution/rd-40-layer-dependency-enforcement.md). Interfaces first because they unlock host-side testing for everything else; renames last so they land on the new structure; enforcement as the guardrail.

---

| ← Previous | Up | Next → |
|---|---|---|
| [PID Motion Control](../algorithms/pid-motion-control.md) | **R&D Items** | [Control & Motion](control/index.md) |
