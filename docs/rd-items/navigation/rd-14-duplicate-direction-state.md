[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-14**

# RD-14 — Two Independent Heading Records Can Silently Desync

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:28`, `API.h:21` (updates in `API.cpp:62,70`)

## Problem

Heading is tracked twice with no reconciliation:

1. The solver's global `currentDirection`, updated inside its own `turn()`.
2. `API`'s private static `currentDirection`, updated inside `turnRight()`/`turnLeft()`.

They agree only because every turn happens to pass through both code paths. Nothing asserts their equality; any future motion path that touches one but not the other (a fast-run shortcut, a recovery maneuver) diverges them silently — after which every absolute↔relative translation in the solver is wrong by a constant offset, and walls get recorded on the wrong side.

## Proposed Approach

Pick one book of record (see [State Management](../../design-decisions/state-management.md)):

- Delete `API`'s hidden copy entirely (simplest), or
- Expose `API::getDirection()` / `getX()/getY()` and make the solver read from it, deleting the globals' heading field.

Add a debug assertion comparing both while migrating, to catch drift early.

## Acceptance Criteria

- [ ] Exactly one heading record exists in the codebase.
- [ ] Full sim run produces identical exploration trace before/after refactor.

Related: [RD-21](../code-health/rd-21-dead-position-state-in-api.md) (the same duplication exists for position).

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-10 Rear wall assumption](rd-10-rear-wall-assumption.md) | [Navigation](index.md) | [RD-17 Unbounded search loop](rd-17-unbounded-search-loop.md) |
