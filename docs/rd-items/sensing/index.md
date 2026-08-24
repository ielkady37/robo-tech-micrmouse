[Home](../../index.md) › **R&D Items** › Sensing

# R&D Items — Sensing

Inertial sensing and time-of-flight ranging: background refresh, calibration, data validity, and sensor identity.

| ID | Priority | Item |
|---|---|---|
| [RD-03](rd-03-imu-background-refresh-skipped.md) | P1 High | Background task calls a getter instead of `imu.update()` — IMU never refreshed there |
| [RD-04](rd-04-imu-calibration-window-broken.md) | P1 High | Calibration window timer never starts; 5 s averaging collects zero samples |
| [RD-09](rd-09-pitch-roll-getter-race.md) | P2 Medium | `getPitch()`/`getRoll()` read shared state without taking the mutex |
| [RD-12](rd-12-zero-distance-read-as-wall.md) | P2 Medium | Zero-initialized ToF cache reads as "wall ahead" / triggers e-stop |
| [RD-13](rd-13-invalid-range-statuses.md) | P2 Medium | All range statuses except out-of-range are accepted as valid distances |
| [RD-30](rd-30-tof-sensor-naming.md) | P3 Low | Sensor numbering (lox1/2/3) contradicts semantic order (right/center/left) |

---

| ← Previous | Up | Next → |
|---|---|---|
| [Control & Motion](../control/index.md) | **Sensing** | [RD-03 IMU background refresh skipped](rd-03-imu-background-refresh-skipped.md) |
