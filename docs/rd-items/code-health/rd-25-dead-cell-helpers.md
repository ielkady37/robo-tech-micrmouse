[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-25**

# RD-25 — Dead Cell-Pack Helpers

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `algorithm.cpp:15-25`

## Problem

Three helpers pack/unpack `(row,col)` into a single byte via bit shifts:

```cpp
uint8_t getCell(uint8_t row, uint8_t col, API api);
uint8_t getRow(uint8_t cell, API api);
uint8_t getCol(uint8_t cell, API api);
```

None is called anywhere. They also declare `API api` parameters ([RD-20](rd-20-api-by-value-signatures.md)) despite doing nothing environment-related. Dead code implies an abandoned design (flat-indexed arrays?) that readers must reverse-engineer to rule out.

## Proposed Approach

Delete them and their declarations in `algorithm.h`. If flat indexing ever returns, reintroduce with tests.

## Acceptance Criteria

- [ ] No uncalled helper functions in the solver.
- [ ] Build clean after removal.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-24 Stubbed drift calibration](rd-24-stubbed-drift-calibration.md) | [Code Health](index.md) | [RD-28 Hardcoded tuning values](rd-28-hardcoded-tuning-values.md) |
