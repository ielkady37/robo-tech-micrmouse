[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-26**

# RD-26 — Wall Sensors Polled Twice per Cell; Recompute Fires Blindly

**Priority:** P3 (Low) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:301-322`

## Problem

Two inefficiencies in the exploration loop's sensing stage:

1. **Double polling**: `isWallInDirection` is called for all four absolutes (each resolving to `wallFront/wallLeft/wallRight`), then the loop immediately calls `api.wallFront() || api.wallLeft() || api.wallRight()` again to decide whether to recompute the distance map. Every cell therefore costs six relative wall queries instead of three.
2. **Imprecise trigger**: the recompute fires when *any* wall is present — including walls already known and recorded. The expensive global map rebuild thus runs on most cells, even no-op ones.

## Proposed Approach

Read each relative sensor exactly once per cell into locals; use them for both wall-marking and change detection. Track whether marking actually added new information (dirty flag — shared with [RD-06](../reliability/rd-06-flash-wear-per-cell-save.md)) and only then call `updateDistancesAStar`.

## Acceptance Criteria

- [ ] Exactly three relative wall queries per cell (log counter).
- [ ] Distance map recomputes only on genuine discoveries; trace shows reduced recompute count with identical final paths.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-17 Unbounded search loop](rd-17-unbounded-search-loop.md) | [Navigation](index.md) | [RD-27 Distance sentinel collision](rd-27-distance-sentinel-collision.md) |
