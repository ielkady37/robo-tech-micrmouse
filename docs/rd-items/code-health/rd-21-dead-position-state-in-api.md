[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-21**

# RD-21 — `API` Maintains Position State Nobody Reads

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `API.cpp:5-8`, `API.cpp:36-57`

## Problem

`API` privately tracks `currentX`, `currentY`, and updates them on every `moveForward()`/`turnRight()`/`turnLeft()` — yet the solver never reads them, keeping its own row/col globals instead ([State Management](../../design-decisions/state-management.md)). This is write-only bookkeeping: real maintenance cost (it must stay correct or silently mislead future features) with zero consumers.

## Proposed Approach

Resolve together with [RD-14](../navigation/rd-14-duplicate-direction-state.md): either delete the position fields from `API`, or promote them to the single source of truth with accessors (`getX()/getY()/getDirection()`) that the solver adopts. Deleting is the smaller change today.

## Acceptance Criteria

- [ ] No write-only pose state remains in `API`.
- [ ] Sim traces unchanged.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-20 API passed by value](rd-20-api-by-value-signatures.md) | [Code Health](index.md) | [RD-22 Unused IMU report members](rd-22-unused-imu-report-members.md) |
