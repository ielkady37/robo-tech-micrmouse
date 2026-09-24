# Turn control and motion results

The IMU has one polling task, started by `IMU::begin()`. It requests 100 Hz
rotation-vector reports and continues sampling during motor braking, delays and
ToF ranging. Motion code consumes `IMUReading` snapshots; it must not call
`imu.update()` concurrently with that task.

Each snapshot contains continuous yaw, a monotonic fresh-report acquisition
timestamp, a receipt time, a sequence number and a calibration flag. The timestamp
extends the local `micros()` clock across rollover and advances only when a new
report is acquired. Driver timestamps can jump or restart, so they are not used
as the PID clock or as a monotonic acceptance gate. A repeated driver
timestamp/sequence pair and invalid quaternions are rejected. A repeated snapshot
cannot update the controller or advance its settling counter.

Startup waits for an actual calibrated, fresh heading rather than assuming a
five-second delay was sufficient. The motors remain stopped while the IMU
calibrates for `IMU::CALIBRATION_MS` (5000 ms). Readiness attempts allow 7000 ms
during calibration, and startup retries if the first usable report is late.
Once ready, normal stale-data detection uses 500 ms. A fresh cached report is
accepted as a baseline; the controller then requires new samples for rate and
settling calculations. Turn deadlines start after the readiness wait, so the
calibration period cannot consume a turn's time budget.

## Settings

These are manual starting values in `Robot.cpp` and `IMU.h`; validate them on the robot.

| Setting | Default | Meaning |
| --- | ---: | --- |
| `IMU::STREAM_TIMEOUT_MS` / `IMU_TIMEOUT_MS` | 500 ms | Shared report-loss and motion-stop threshold; bounds readiness waits after calibration |
| `IMU_STARTUP_TIMEOUT_MS` | 7000 ms | One readiness attempt while calibration is in progress; startup retries while stopped |
| `MOTION_TIMEOUT_MS` | 2000 ms | No encoder movement triggers one recovery attempt |
| `TURN_TIMEOUT_MS` | 3000 ms | Turn deadline for up to 90°; scales with larger requested corrections |
| `ERROR_TOL` | 1° | Settled heading tolerance |
| `RATE_TOL` | 5°/s | Maximum raw and filtered rate for settling |
| `TURN_STABLE_SAMPLES` | 5 | Minimum consecutive distinct settled measurements |
| `TURN_SETTLE_US` | 50000 µs | Minimum settled fresh-report time span; at 100 Hz this requires six samples |
| `TURN_FINE_ANGLE` | 10° | Switch from continuous PID drive to short correction pulses |
| `TURN_PULSE_MS` | 20 ms | Maximum correction pulse duration |
| `TURN_PULSE_BRAKE_MS` | 40 ms | Minimum braking time between pulses; also wait for measured rate to settle |
| `TURN_RATE_FILTER_SEC` | 0.04 s | Rotation-rate low-pass filter time constant |

Pulses use the fixed `MIN_SPEED_ROT` PWM. Normal commands remain bounded by the
hardcoded motor limits. Pulse duration, stale-IMU checks and turn deadlines are
checked even between sensor reports. Wheel movement cannot extend a turn's
absolute deadline. The deadline bounds the control phase; stopping/settling and
a stall recovery have their own bounded durations.

## Targets and navigation

- `Robot::turn(angle)` is a relative-angle command for manual sequences.
- `Robot::turnCardinal(n)` targets the last confirmed maze heading plus `n * 90°`.
  `API::turnLeft()` and `API::turnRight()` use this command.
- `turnCardinal(0)` restores the last confirmed maze heading. Navigation uses it
  before reading walls at the start of a flood-fill pass.
- `snapToCardinal()` selects the nearest multiple of 90° from a fresh heading.

`MotionResult` is returned through Robot, API and the navigation motion helpers:

| Result | Meaning |
| --- | --- |
| `Completed` | Requested motion completed |
| `Stalled` | Encoders stopped progressing; one backup-and-align recovery was attempted |
| `ImuTimeout` | No trustworthy fresh heading was available; motors stopped |
| `TurnTimeout` | Turn exceeded its absolute deadline; motors stopped |
| `Blocked` | Forward movement stopped early for a close front wall |

Only `Completed` advances stored coordinates/headings. Each successful quarter
turn is recorded separately, so a failure during the second half of a U-turn
retains the first completed turn. A failed turn prevents its following forward
move. Flood fill returns on a motion failure before advancing the cell or saving
an unconfirmed move.

Recovery still backs up using the configured `RECOVERY_BACKUP_CM` (currently
4 cm), then attempts nearest-cardinal alignment. It requires fresh IMU samples
and cannot recursively start another recovery. Recovery does not convert the
original failure to `Completed` or change the intended maze heading. Stored cell
coordinates remain the last confirmed cell; this is not localization of a
partially completed move or of the backup displacement.

## IMU stream recovery

The sampling task detects the BNO08x reset notification and prolonged absence of
accepted reports. It invalidates the cached heading and increments its stream
generation. Moves and turns abort with `ImuTimeout` if that generation changes,
even if the IMU recovers before the next control iteration. The motors brake,
and the interrupted command remains incomplete.

Recovery uses `enableReport(SH2_ARVR_STABILIZED_RV, 10000)`, entirely within the
existing sampling task. The event buffer has persistent lifetime because the
Adafruit driver retains its pointer and can deliver callbacks during report
configuration. Driver calls never hold the snapshot mutex. `begin()` is
idempotent, so it cannot create another polling task.

| Setting in `IMU.h` | Default | Meaning |
| --- | ---: | --- |
| `MAX_RECOVERY_ATTEMPTS` | 3 | Maximum report-restart requests per outage |
| `RECOVERY_RETRY_MS` | 500 ms | Minimum spacing between attempts; also the final confirmation window after attempt 3 |
| `RECOVERY_CONFIRM_REPORTS` | 5 | Consecutive fresh accepted reports needed to restore readiness |
| `RECOVERY_CONFIRM_SPAN_MS` | 40 ms | Minimum time covered by the confirmation reports |
| `RECOVERY_CONFIRM_GAP_MS` | 100 ms | Larger report gaps restart confirmation |

A configuration acknowledgement alone does not restore readiness. Duplicate or
invalid reports reset confirmation. After attempts are exhausted, state becomes
`Failed`, configuration retries stop and the sampler keeps listening. A later
confirmed stream can return to `Ready` without restarting the ESP32. Repeated
sensor resets within an unresolved outage do not replenish the retry budget.
Report recovery preserves software calibration, yaw offset, the acquisition
clock, and the intended maze heading; it does not perform another five-second
calibration or mark the interrupted motion complete.

Navigation polls readiness while paused, rather than repeatedly executing and
logging the same failed alignment. When the stream is confirmed, it prints
`Retrying heading alignment` and aligns to the last confirmed maze heading
before interpreting walls. Other motion failures retain their result handling.

### Failure diagnostics

Logs identify reset detection, stream loss, each restart attempt, exhausted
attempts and confirmed recovery. `IMU status` includes:

- `state`, stream `generation`, and `sampleAgeMs` (`none` before a heading exists).
- `pollAgeMs`: time since the sampler began its most recent driver poll. A large
  value can indicate a blocked driver call or delayed task scheduling.
- `received` and `accepted` report counts; accepted includes calibration reports.
- `duplicate`, `invalid`, and `other` report counts.
- `resets`, per-outage `attempts`, cumulative `requestFailures`, and `maxGapMs`.

Motion fault logs distinguish stale headings, stream restarts and invalid sample
intervals. Repeated `ImuTimeout` messages are suppressed until a motion completes;
stream transition and recovery-attempt logs still identify each outage.

## Host checks

From the repository root, with an existing `/tmp/opencode` directory:

```sh
g++ -std=c++17 -Wall -Wextra -Itests/host tests/imu_samples.cpp -o /tmp/opencode/imu-samples-test
/tmp/opencode/imu-samples-test
g++ -std=c++17 -Wall -Wextra -Itests/host tests/motion_control.cpp -o /tmp/opencode/motion-control-test
/tmp/opencode/motion-control-test
g++ -std=c++17 -Wall -Wextra -Itests/host tests/startup_calibration.cpp -o /tmp/opencode/startup-calibration-test
/tmp/opencode/startup-calibration-test
g++ -std=c++17 -Wall -Wextra -Itests/host tests/imu_recovery.cpp -o /tmp/opencode/imu-recovery-test
/tmp/opencode/imu-recovery-test
```

These compile the production IMU/control/navigation code against simulated
hardware. They cover real calibration-to-motion startup, delayed first reports,
driver-clock restarts, recovery after a timeout, sample freshness, false settling, sensor
loss, frozen heading with moving encoders, turn deadlines, fixed cardinal and
relative targets, fine pulses, bounded recovery, persistent callback storage,
reset storms, recovery confirmation, paused navigation and propagation of failures
through navigation. They do not validate ESP32 scheduling, physical braking,
motor tuning or sensor accuracy.
