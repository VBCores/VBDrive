# Control modes

## Purpose

Define the common Servo and MIT command semantics, controller laws, command identity and rejection behavior used by Serial and Cyphal.

## Requirements

### Requirement: Servo mode identifiers

Serial and Cyphal SHALL interpret the same Servo IDs and scalar setpoints:

| ID | Mode | Setpoint |
| --- | --- | --- |
| 0 | VELOCITY_DIRECT | Output rad/s |
| 1 | VELOCITY_RAMP | Output rad/s |
| 2 | TORQUE_DIRECT | Output N m |
| 3 | POSITION_DIRECT | Output rad |
| 4 | POSITION_FILTER | Output rad |
| 5 | POSITION_POLY | Output rad |
| 6 | VOLTAGE_DIRECT | Signed user-frame volts |

Unknown IDs and non-finite targets SHALL be rejected. DIRECT modes SHALL bypass reference generation. A generator mode SHALL require its necessary configuration to be strictly positive before acceptance.

#### Scenario: Unconfigured generator

- **WHEN** POSITION_FILTER is requested with zero bandwidth while a direct command is active
- **THEN** the request fails and the active direct command remains unchanged.

### Requirement: Position Servo law

Position Servo SHALL compute output-shaft torque using τ_raw = Kp,p(θ_ref−θ) + Ki,p∫(θ_ref−θ)dt − Kd,p ω. The D term SHALL use measured user-frame velocity. Generated reference velocity SHALL NOT be injected as a separate feed-forward component. The scalar position reference SHALL come from the active direct or generated position input.

#### Scenario: Position target step

- **WHEN** the reference position changes with unchanged measured velocity
- **THEN** the P term changes immediately while D contains no derivative of that target step.

### Requirement: Velocity Servo law and saturation

Velocity Servo SHALL compute τ_raw = Kp,v(ω_ref−ω) + Ki,v∫(ω_ref−ω)dt. Position and velocity controllers SHALL be alternative modes, not a cascaded position-to-velocity controller. Servo torque SHALL be clamped each control tick by effective torque and current-derived bounds, including ratings, user limits, stall derating and the board cap. Integral updates SHALL block accumulation further into saturation and allow unwinding. Changing active Servo gains SHALL reset that regulator's integral state; changing only a continuing target SHALL preserve it.

#### Scenario: Saturated controller unwinds

- **WHEN** output is saturated and the integral increment acts toward the unsaturated range
- **THEN** the increment is allowed subject to integral bounds while final torque remains clamped.

### Requirement: MIT impedance law

MIT SHALL compute τ_raw = Kp(θ_cmd−θ) + Kd(ω_cmd−ω) + τ_ff each control tick using gains supplied by the command. MIT SHALL have no Servo integral or trajectory generator. Its summed torque SHALL be converted using the coordinate contract and clipped by the common FOC current limits. All five command values SHALL be finite; position, velocity and feed-forward torque SHALL pass their respective target checks before the command replaces active state.

#### Scenario: Pure torque feed-forward

- **WHEN** a valid MIT command has Kp=Kd=0
- **THEN** the requested torque is τ_ff before the common current clipping.

### Requirement: Direct torque and voltage paths

TORQUE_DIRECT SHALL convert the validated torque target into Iq_ref and use current regulation. VOLTAGE_DIRECT SHALL set Vd=0 and Vq=s V_cmd, bypassing the outer Servo controller and current regulators while retaining voltage-vector and PWM limits. Current-reference clipping SHALL NOT be described as current regulation in VOLTAGE_DIRECT.

#### Scenario: Direct voltage

- **WHEN** a finite VOLTAGE_DIRECT target is accepted
- **THEN** voltage is the controlled input and the d/q current-error regulators are bypassed.

### Requirement: Shared command identity

The last accepted Servo command SHALL be shared by Serial and Cyphal. Without an index, consecutive identical mode/value commands SHALL be successful repeats. With an index, the same index and content SHALL be a successful repeat; the same index with changed content SHALL be rejected. A different index, including 255→0, or addition/removal of an index SHALL identify a new command. A repeat SHALL NOT restart a profile. Rejection SHALL preserve the previous accepted command and active reference; a malformed or rejected movement target processed in RUNNING SHALL increment cmd_errors once per request.

#### Scenario: Cross-transport repeat and conflict

- **WHEN** Serial accepts POSITION_POLY value 1 with index 255, followed by Cyphal with the same content and index
- **THEN** the profile continues without restart
- **AND** a later index-255 request with value 2 is rejected and increments cmd_errors once.

### Requirement: Control lifecycle resets

STOP, OFF, calibration and an accepted MIT transition SHALL clear Servo command history and generator state. A subsequent Servo command SHALL be new even if identical to the command preceding that reset. Motion commands SHALL NOT implicitly enable an OFF bridge; commands accepted while OFF SHALL NOT be replayed automatically on enable.

#### Scenario: Reuse an index after STOP

- **WHEN** STOP follows a Servo command with index 7 and that command is sent again
- **THEN** it is treated as a new command rather than a duplicate from before STOP.
