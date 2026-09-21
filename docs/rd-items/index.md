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

**Software-first**: ordered so everything runnable against the simulator/host build lands before anything that needs the physical robot; Low-priority items are deferred to last regardless of type. The **Infra** column in each subsystem table below marks each item **Agnostic** (simulator/host-testable) or **Dependent** (needs the real ESP32 + motors/IMU/ToF/flash/switch).

**Phase 1 — Critical & High, infra-agnostic** (simulator/host only, no hardware needed):

1. [RD-05](navigation/rd-05-fallback-move-north-oob.md) — solver fallback to NORTH can leave the maze (P1)
2. [RD-35](design-evolution/rd-35-hardware-abstraction-layer.md) — HAL: sensor & actuator interfaces (P1)
3. [RD-36](design-evolution/rd-36-environment-seam-sim-real.md) — environment seam: simulator ↔ real hardware (P1)

**Phase 2 — Medium, infra-agnostic**:

4. [RD-10](navigation/rd-10-rear-wall-assumption.md), [RD-11](navigation/rd-11-return-phase-unimplemented.md), [RD-14](navigation/rd-14-duplicate-direction-state.md), [RD-17](navigation/rd-17-unbounded-search-loop.md) — navigation correctness
5. [RD-18](code-health/rd-18-static-api-singleton.md) — `API` singleton limits testing
6. [RD-37](design-evolution/rd-37-application-services-navigator.md), [RD-38](design-evolution/rd-38-solver-strategy-seam.md) — Navigator/domain model, pluggable solver

**Phase 3 — Critical, infra-dependent** (first item that needs the real robot):

7. [RD-01](control/rd-01-heading-correction-disabled.md) — heading correction ×0

**Phase 4 — High, infra-dependent**:

8. [RD-07](control/rd-07-full-speed-drift-calibration.md) — full-speed drift calibration (safety first)
9. [RD-03](sensing/rd-03-imu-background-refresh-skipped.md), [RD-04](sensing/rd-04-imu-calibration-window-broken.md) — IMU background refresh + calibration window
10. [RD-02](control/rd-02-turn-feedforward-prevents-settling.md) — turn feed-forward prevents settling
11. [RD-06](reliability/rd-06-flash-wear-per-cell-save.md) — flash wear from per-cell saves

**Phase 5 — Medium, infra-dependent**:

12. [RD-12](sensing/rd-12-zero-distance-read-as-wall.md), [RD-13](sensing/rd-13-invalid-range-statuses.md) — ToF reading validity
13. [RD-09](sensing/rd-09-pitch-roll-getter-race.md) — pitch/roll getter race
14. [RD-08](control/rd-08-pid-signal-truncation.md), [RD-16](control/rd-16-busy-wait-sensor-task.md) — PID truncation, busy-wait sensor task
15. [RD-15](reliability/rd-15-hang-on-init-failure.md), [RD-19](reliability/rd-19-mode-switch-polarity.md) — init hang, switch polarity

**Phase 6 — Low priority** (deferred regardless of infra dependency; agnostic first):

- Agnostic: [RD-20](code-health/rd-20-api-by-value-signatures.md), [RD-21](code-health/rd-21-dead-position-state-in-api.md), [RD-25](code-health/rd-25-dead-cell-helpers.md), [RD-26](navigation/rd-26-redundant-wall-readings.md), [RD-27](navigation/rd-27-distance-sentinel-collision.md), [RD-31](code-health/rd-31-dead-code-in-sketch.md), [RD-39](design-evolution/rd-39-domain-rename-taxonomy.md), [RD-40](design-evolution/rd-40-layer-dependency-enforcement.md)
- Dependent: [RD-29](control/rd-29-hardcoded-motor-trim.md), [RD-30](sensing/rd-30-tof-sensor-naming.md), [RD-22](code-health/rd-22-unused-imu-report-members.md), [RD-23](code-health/rd-23-unused-sparkfun-include.md), [RD-24](code-health/rd-24-stubbed-drift-calibration.md), [RD-28](code-health/rd-28-hardcoded-tuning-values.md), [RD-34](code-health/rd-34-threshold-units-undocumented.md), [RD-32](code-health/rd-32-debug-string-quality.md), [RD-33](code-health/rd-33-commented-legacy-code.md)

---

| ← Previous | Up | Next → |
|---|---|---|
| [PID Motion Control](../algorithms/pid-motion-control.md) | **R&D Items** | [Control & Motion](control/index.md) |
