# Control timing

## Purpose

Define the nominal control schedule and communication servicing contract of VBDrive, including the distinction between scheduled work and measured throughput.

## Requirements

### Requirement: PWM-synchronous control cadence

The controller SHALL schedule one control opportunity per 25 us PWM period (40 kHz). Current sampling SHALL precede control, with a nominal 4 us phase offset between the sampling trigger and control entry. New PWM compare values SHALL take effect at the next PWM period boundary. Current regulation, MIT and active Servo P/D SHALL execute at this cadence while RUNNING, enabled and outside bootstrap charging or service operations.

#### Scenario: Enabled steady-state control

- **WHEN** an initialized, calibrated drive runs an enabled control mode
- **THEN** the nominal PWM and control period is 25 us, with current sampling before the control calculation
- **AND** current regulation and outer-loop P/D are not deliberately decimated to accommodate communications.

### Requirement: Independent reference and integral schedules

An active Servo generator SHALL advance by 200 us every eighth FOC tick, with its scalar reference held between updates. Servo integral SHALL accumulate each 25 us error sample and commit every fifth FOC tick (125 us, 8 kHz). The dividers SHALL be independent. With Ki=0, integral accumulation SHALL be skipped. Continuing goals and duplicate commands SHALL NOT restart either divider; controller resets and mode-entry behavior are defined by the motion and lifecycle contracts.

#### Scenario: Forty steady-state Servo ticks

- **WHEN** an already active generator executes forty FOC ticks without a mode change
- **THEN** it performs five reference updates and eight integral commit opportunities
- **AND** changing the goal between ticks does not change those ratios.

### Requirement: Bounded foreground communications

Cyphal SHALL service at most one RX frame and a bounded TX pass per normal foreground iteration, without waiting for a busy hardware TX FIFO. It SHALL use the newest retained hardware RX data on FIFO overflow, without an additional software queue of stale control commands. A handler invocation SHALL follow completion of a transfer, not every received frame. Ordinary Serial parsing SHALL run in foreground and dispatch at most one complete command per millisecond service opportunity; actual throughput also depends on baud rate and TX readiness.

#### Scenario: Transmitter temporarily busy

- **WHEN** hardware cannot accept the next Cyphal TX frame
- **THEN** servicing returns to the foreground loop so later RX and control work can proceed.

#### Scenario: Multi-frame input

- **WHEN** a transfer requires multiple CAN frames
- **THEN** multiple RX passes can be required before one subscription handler call; loop frequency alone does not prove command throughput.

### Requirement: Fresh periodic state publication

When a motor instance and Cyphal are available, State SHALL be scheduled once per new millisecond (nominally 1000 messages/s), using a fresh measurement snapshot. Missed publication slots SHALL be skipped instead of replayed in a burst. Serial logging SHALL have a nominal 10 ms interval and SHALL be available only in RUNNING.

#### Scenario: Delayed foreground iteration

- **WHEN** the foreground loop resumes after several missed milliseconds
- **THEN** it queues one fresh State sample rather than one sample for every missed slot.

### Requirement: Timing qualification scope

Timing verification SHALL distinguish configured periods, executed FOC ticks, completed command handlers and State messages actually transmitted. Qualification of a smooth 1000-command/s input SHALL measure handler intervals and sender/arrival pacing separately, including changing POLY goals and nonzero Servo I. EEPROM operations, configuration service, calibration, startup and reboot SHALL be excluded from steady-state measurements. Occasional long sensor updates SHALL be assessed with rolling one-second control rates as well as interval distributions.

#### Scenario: Communication performance report

- **WHEN** performance is evaluated under simultaneous control and command traffic
- **THEN** the report identifies firmware build, mode, parameters, traffic pacing and maintenance exclusions
- **AND** a host test or nominal timer frequency is not reported as a physical throughput measurement.
