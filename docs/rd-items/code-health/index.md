[Home](../../index.md) › **R&D Items** › Code Health

# R&D Items — Code Health

Structure, dead weight, and maintainability. Lower urgency individually, but several of these (API shape, magic numbers) remove friction that slows every other fix.

| ID | Priority | Item |
|---|---|---|
| [RD-18](rd-18-static-api-singleton.md) | P2 Medium | All-static `API` singleton prevents testing/extension |
| [RD-20](rd-20-api-by-value-signatures.md) | P3 Low | Algorithm signatures take `API api` by value |
| [RD-21](rd-21-dead-position-state-in-api.md) | P3 Low | `API` maintains position state nobody reads |
| [RD-22](rd-22-unused-imu-report-members.md) | P3 Low | Unused/inconsistent IMU report-type members |
| [RD-23](rd-23-unused-sparkfun-include.md) | P3 Low | Unused SparkFun BNO080 include alongside active Adafruit driver |
| [RD-24](rd-24-stubbed-drift-calibration.md) | P3 Low | Drift-factor feature stubbed out; result never used |
| [RD-25](rd-25-dead-cell-helpers.md) | P3 Low | Dead `getCell`/`getRow`/`getCol` helpers |
| [RD-28](rd-28-hardcoded-tuning-values.md) | P3 Low | PID gains and limits scattered as unnamed magic numbers |
| [RD-31](rd-31-dead-code-in-sketch.md) | P3 Low | 73 lines of commented-out sketch history |
| [RD-32](rd-32-debug-string-quality.md) | P3 Low | Non-descriptive debug string in boot path |
| [RD-33](rd-33-commented-legacy-code.md) | P3 Low | ~180 lines of commented-out legacy implementations |
| [RD-34](rd-34-threshold-units-undocumented.md) | P3 Low | Threshold macros lack unit documentation |

---

| ← Previous | Up | Next → |
|---|---|---|
| [Reliability](../reliability/index.md) | **Code Health** | [RD-18 Static API singleton](rd-18-static-api-singleton.md) |
