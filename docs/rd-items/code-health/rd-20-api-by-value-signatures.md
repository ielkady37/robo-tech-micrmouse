[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-20**

# RD-20 — Algorithm Signatures Take `API api` by Value

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `algorithm.h:2-14`

## Problem

Every solver function declares an `API` parameter by value:

```cpp
void floodFill(API api);
Direction getNextMovement(uint8_t currentRow, uint8_t currentCol, bool returning, API api);
```

Because all of `API`'s members are static ([RD-18](rd-18-static-api-singleton.md)), nothing is actually copied and behavior is unaffected — but the signature lies about there being instance state, blocks introducing any later, and trains readers to ignore parameter meaning.

## Proposed Approach

Either pass by reference (`API& api`) as a minimal honest fix, or — better, after resolving RD-18/RD-21 — drop the parameter entirely once `API` becomes a namespace.

## Acceptance Criteria

- [ ] No by-value `API` parameters remain (or the parameter is gone).
- [ ] Pure signature refactor; identical sim traces.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-18 Static API singleton](rd-18-static-api-singleton.md) | [Code Health](index.md) | [RD-21 Dead position state in API](rd-21-dead-position-state-in-api.md) |
