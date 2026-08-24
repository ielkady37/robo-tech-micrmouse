[Home](../../index.md) › **R&D Items** › Design Evolution

# R&D Items — Design Evolution

Structural improvements proposed from the design review: layering, seams, naming, and guardrails. Unlike the defect sections above, these items reshape what the codebase *is*; several unblock the testing and simulation work the defects depend on.

| ID | Priority | Item |
|---|---|---|
| [RD-35](rd-35-hardware-abstraction-layer.md) | P1 High | Hardware abstraction layer: sensor & actuator interfaces |
| [RD-36](rd-36-environment-seam-sim-real.md) | P1 High | Environment seam: simulator ↔ real hardware via `IMazeEnvironment` |
| [RD-37](rd-37-application-services-navigator.md) | P2 Medium | Application services layer: `Navigator` + domain model (`MazeMap`, `Pose`) |
| [RD-38](rd-38-solver-strategy-seam.md) | P2 Medium | Algorithm strategy seam: pluggable `IMazeSolver` (A* and alternatives) |
| [RD-39](rd-39-domain-rename-taxonomy.md) | P3 Low | Domain-specific rename taxonomy (`API`/`Robot` out, maze language in) |
| [RD-40](rd-40-layer-dependency-enforcement.md) | P3 Low | Layer dependency enforcement (keep the layers honest) |

## Sequencing

Deliberate order: **RD-35 → RD-36 → RD-37 → RD-38 → RD-39 → RD-40**.

Interfaces first (they unblock sim/real and offline tests), then the service/domain extraction, then the strategy seam, then renames — renaming *last* avoids churning against a structure about to change — and enforcement as the final guardrail. See the [target architecture](../../overview/index.md#the-solution-shape) these items converge on.

---

| ← Previous | Up | Next → |
|---|---|---|
| [Code Health](../code-health/index.md) | **Design Evolution** | [RD-35 Hardware abstraction layer](rd-35-hardware-abstraction-layer.md) |
