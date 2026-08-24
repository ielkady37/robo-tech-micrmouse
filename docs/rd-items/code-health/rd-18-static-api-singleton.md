[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-18**

# RD-18 — All-Static `API` Singleton Limits Testing and Extension

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Code Health · **Source:** `API.h:16-52`

## Problem

Every member of `API` is `static`. The class is a namespace in disguise:

- It cannot be instantiated, so there can be exactly one environment forever.
- Mocking it for solver unit tests requires linker tricks or source edits rather than dependency injection.
- The by-value `API api` parameters sprinkled through `algorithm.h` are rendered meaningless ([RD-20](rd-20-api-by-value-signatures.md)).
- Its hidden pose state is a side effect of the same design ([RD-21](rd-21-dead-position-state-in-api.md)).

## Proposed Approach

Pick one shape deliberately:

1. **Namespace**: rename to `namespace api { ... }`, delete parameters, keep statics. Minimal churn, honest semantics.
2. **Instance + interface**: define an `IEnvironment` abstract class, make `SimulatorApi` and `RobotAdapter` implement it, inject into the solver. Enables mocks in tests and formalizes the [dual-target workflow](../../design-decisions/dual-target-workflow.md).

Option 2 pays for itself the first time solver regressions need automated testing.

## Acceptance Criteria

- [ ] One of the two shapes implemented consistently; no mixed statics/instances.
- [ ] Solver compiles against both adapters without source edits (option 2).

---

| ← Previous | Up | Next → |
|---|---|---|
| [Code Health](index.md) | [Code Health](index.md) | [RD-20 API passed by value](rd-20-api-by-value-signatures.md) |
