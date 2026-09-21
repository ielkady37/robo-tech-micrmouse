[Home](../../index.md) › **R&D Items** › Navigation

# R&D Items — Navigation

Maze-solver safety and correctness: bounds, direction state, termination, and the return trip.

| ID | Priority | Infra | Item |
|---|---|---|---|
| [RD-05](rd-05-fallback-move-north-oob.md) | P1 High | Agnostic | No improving neighbor falls back to NORTH unchecked → out-of-bounds / collision |
| [RD-10](rd-10-rear-wall-assumption.md) | P2 Medium | Agnostic | Rear-direction wall query silently answers "no wall" from a wrong comment's assumption |
| [RD-11](rd-11-return-phase-unimplemented.md) | P2 Medium | Agnostic | Return-to-start phase never activates (`returning` hardcoded false) |
| [RD-14](rd-14-duplicate-direction-state.md) | P2 Medium | Agnostic | Two independent heading records can silently desync |
| [RD-17](rd-17-unbounded-search-loop.md) | P2 Medium | Agnostic | Exploration loop has no iteration cap — trapped states spin forever |
| [RD-26](rd-26-redundant-wall-readings.md) | P3 Low | Agnostic | Wall sensors polled twice per cell; recompute fires even without new walls |
| [RD-27](rd-27-distance-sentinel-collision.md) | P3 Low | Agnostic | `uint8_t` distance array shares values with the UINT8_MAX infinity sentinel |

---

| ← Previous | Up | Next → |
|---|---|---|
| [Sensing](../sensing/index.md) | **Navigation** | [RD-05 Fallback move NORTH drives out of bounds](rd-05-fallback-move-north-oob.md) |
