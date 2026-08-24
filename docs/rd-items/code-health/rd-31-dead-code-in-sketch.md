[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-31**

# RD-31 — 73 Lines of Commented-Out Sketch History

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `micromouse.ino:1-73`

## Problem

The top of the entry-point file is an entire commented-out sketch — the old `Robot`-based choreography of hardcoded moves. The active program doesn't begin until line 76, and the file's real structure (switch check → `loadMatrix()` → `floodFill` loop) is buried under 73 lines of history.

## Proposed Approach

Delete the block; git preserves it. Optionally leave a one-line comment noting that the hardware path lives in `Robot.*` and is wired via [Dual-Target Workflow](../../design-decisions/dual-target-workflow.md) when needed.

## Acceptance Criteria

- [ ] `micromouse.ino` contains only live code plus intentional comments.
- [ ] Entry point is readable within one screen.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-28 Hardcoded tuning values](rd-28-hardcoded-tuning-values.md) | [Code Health](index.md) | [RD-32 Debug string quality](rd-32-debug-string-quality.md) |
