[Home](../../index.md) › **R&D Items** › Control & Motion

# R&D Items — Control & Motion

Closed-loop movement: straight-line heading hold, turn settling, scheduling of the control task, and calibration/tuning constants.

| ID | Priority | Item |
|---|---|---|
| [RD-01](rd-01-heading-correction-disabled.md) | **P0** Critical | Heading correction multiplied by zero — straight-line hold is inert |
| [RD-02](rd-02-turn-feedforward-prevents-settling.md) | P1 High | Unconditional ±15 feed-forward overrides turn deadband → oscillation |
| [RD-07](rd-07-full-speed-drift-calibration.md) | P1 High | Drift calibration drives both motors at 255 for 5 s with no safety |
| [RD-08](rd-08-pid-signal-truncation.md) | P2 Medium | `int pidSignal` truncates float PID output before scaling |
| [RD-16](rd-16-busy-wait-sensor-task.md) | P2 Medium | `vTaskDelay(0)` busy-waits instead of yielding CPU |
| [RD-29](rd-29-hardcoded-motor-trim.md) | P3 Low | Undocumented hardcoded ×0.98 left-motor PWM trim |

---

| ← Previous | Up | Next → |
|---|---|---|
| [R&D Items](../index.md) | **Control & Motion** | [RD-01 Heading correction disabled](rd-01-heading-correction-disabled.md) |
