# Coordinates and units

## Purpose

Define the units and sign conventions shared by commands, reference generation, feedback, FOC conversion and telemetry.

## Requirements

### Requirement: User-facing physical units

Commands, user limits, generated references and reported output-shaft state SHALL use radians, radians/second, radians/second squared and newton metres. Voltage and current SHALL use volts and amperes. `kt` SHALL be output-shaft N m/A, generator bandwidth SHALL be s^-1 without a 2π multiplier, and temperature registers SHALL report kelvins. Encoder registers SHALL report raw counts. The Cyphal State timestamp SHALL be in microseconds; the current timestamp is local device time, without network clock synchronization.

#### Scenario: Temperature and state interpretation

- **WHEN** a client reads a temperature of 303.15 and velocity of 2
- **THEN** these denote 30 degrees Celsius and 2 output-shaft rad/s, independently of the transport.

### Requirement: Direction and offset transform

`ang_dir` SHALL accept only +1 or -1, selecting CCW-positive or CW-positive output motion respectively. Let s = -ang_dir relative to VBDrive's CW-positive native frame. Position, velocity and torque feedback SHALL be transformed as θ = s θ_native + ang_off, ω = s ω_native and τ = s τ_native. Position and velocity targets and angle limits SHALL use this same user frame. Changing `ang_off` SHALL translate position without translating velocity or torque.

#### Scenario: Negative native rotation with CCW selected

- **WHEN** ang_dir=+1, ang_off=0 and native output velocity is negative
- **THEN** reported user velocity is positive and reported position increases.

#### Scenario: Position offset

- **WHEN** native output position is 0.2 rad, ang_dir=+1 and ang_off=0.5 rad
- **THEN** reported user position is 0.3 rad.

### Requirement: Mechanical and electrical frames remain distinct

Rotor observer state SHALL use mechanical rotor radians, rad/s and rad/s² before the gearbox. The mechanical rotor-to-output position and velocity conversion SHALL divide by `gear`; electrical phase SHALL multiply rotor mechanical phase by the pole-pair count. Electrical phase SHALL remain in the native motor frame and SHALL NOT receive user `ang_off` or `ang_dir` corrections.

#### Scenario: One rotor revolution

- **WHEN** rotor position advances by 2π mechanical radians with gear ratio G
- **THEN** native rotor-derived output position advances by 2π/G radians
- **AND** electrical phase advances through the number of periods set by pole pairs.

### Requirement: Position and velocity feedback sources

With rotor position selected, output position SHALL unwrap the encoder angle before observer filtering and divide by gear. With shaft position selected, output position SHALL use the shaft sensor's revolutions and angle. Output velocity SHALL use predicted rotor mechanical velocity divided by gear in either selection. The direction/offset transform SHALL then apply to user-facing feedback.

#### Scenario: Shaft encoder selection

- **WHEN** `ang_enc=1`
- **THEN** reported position uses the shaft sensor while reported velocity remains rotor-observer-derived.

### Requirement: Torque conversion includes transmission in kt

Torque/current conversion SHALL use Iq_ref = s τ_user / kt and τ_user = s kt Iq. It SHALL NOT multiply or divide by gear again. Positive and negative torque commands SHALL use the same convention in MIT, Servo torque output and TORQUE_DIRECT. Current-derived torque limits SHALL use the same kt.

#### Scenario: Gear-independent torque conversion

- **WHEN** kt=0.5 N m/A and a user torque of 1 N m is requested
- **THEN** the unclipped current reference magnitude is 2 A regardless of the configured gear ratio
- **AND** its native sign is determined by s.

### Requirement: Target limits use user coordinates

Finite `min_ang` and `max_ang` SHALL independently bound position targets inclusively. A positive finite `max_spd` SHALL bound velocity targets symmetrically; it SHALL NOT act as a measured-speed clamp in position mode. NaN SHALL clear the respective user angle or speed bound. A non-finite motion target SHALL remain invalid even when its limit is unset.

#### Scenario: Unset speed bound

- **WHEN** max_spd is NaN
- **THEN** finite velocity targets are not rejected by that user speed bound, while a NaN target is rejected.
