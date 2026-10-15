# Trajectory generation

## Purpose

Define the reference generators that shape scalar Servo inputs while leaving feedback tracking to the selected Servo controller.

## Requirements

### Requirement: Autonomous reference evolution

Reference generators SHALL operate in user output-shaft coordinates using configuration, target, state and elapsed time. Once initialized, normal progression SHALL NOT be corrected from subsequent measured position or velocity. Generated position or velocity SHALL be the sole scalar Servo setpoint, without additional feed-forward. Active reference state SHALL be published coherently without heap allocation; command preparation SHALL NOT roll back progress made by a concurrent control tick.

#### Scenario: Stationary measured shaft with changing goals

- **WHEN** an active position generator receives successive goals while the measured shaft remains stationary
- **THEN** its reference continues from its own state instead of repeatedly returning to measured position.

### Requirement: Mode-entry initial state

On entry to FILTER or POLY, initial position SHALL be measured user position. Initial velocity SHALL be the previous generator velocity when available and its absolute difference from measured user velocity is at most velocity_planning_tolerance; otherwise it SHALL be measured velocity. The default tolerance SHALL be 0.5 rad/s, with equality accepted. RAMP SHALL initialize from measured velocity at its first scheduled activation. Continuing the same active mode SHALL NOT apply tolerance-driven resets.

#### Scenario: Velocity outside entry tolerance

- **WHEN** a position-generator mode is entered with measured velocity 0.1 rad/s, previous reference velocity 1 rad/s and tolerance 0.5 rad/s
- **THEN** initial position and velocity come from measurements.

#### Scenario: Continuing mode exceeds tolerance

- **WHEN** an active POLY reference differs from measured velocity by more than tolerance and receives a new goal
- **THEN** planning continues from the previous reference position and velocity.

### Requirement: Critically damped position filter

FILTER SHALL use b = min(servo_control_input_bandwith, 0.25/dt), a = b²(goal−p_ref)−2b v_ref, followed by v_ref += a dt and p_ref += v_ref dt, in that order. At dt=200 us the bandwidth cap SHALL be 1250 s^-1. A new goal or valid bandwidth update SHALL preserve the latest reference position and velocity. FILTER SHALL continue numerical filtering without an artificial completion threshold.

#### Scenario: New filter target during a step

- **WHEN** a FILTER command is prepared and the control loop advances before its publication
- **THEN** the published target/configuration is combined with the latest filter state rather than an older copy.

### Requirement: Piecewise quadratic position profile

POSITION_POLY SHALL generate piecewise constant acceleration, piecewise linear velocity and piecewise quadratic position. Its positive velocity, acceleration and deceleration limits SHALL define a trapezoidal profile or a triangular profile for a short move, ending at the target with zero reference velocity. An initial velocity away from the target, above cruise speed or unable to stop before the target SHALL be conditioned by braking, including overshoot and return when needed. An already excessive initial speed SHALL NOT be instantaneously clipped to cruise speed.

#### Scenario: Insufficient stopping distance

- **WHEN** initial velocity cannot be stopped before the requested position
- **THEN** the profile brakes with the configured deceleration, passes the goal if necessary and returns to finish at that goal with zero reference velocity.

#### Scenario: Profile completion

- **WHEN** profile time reaches or exceeds its finish time
- **THEN** position is exactly the target and velocity is zero, including a zero-distance move from rest.

### Requirement: Continuous POLY retargeting and time alignment

A continuing POLY command or valid parameter update SHALL plan from the previous reference position, velocity and associated time. Planning SHALL occur outside long interrupt-disabled sections, followed by a coherent short publication. The time from the snapshot to the next reference tick and its 200 us reference horizon SHALL each be accounted for once. Until that tick, the existing PID input SHALL be held. Retargeting SHALL preserve position and velocity continuity at the join; acceleration may change with the new goal and limits.

#### Scenario: Retarget between reference ticks

- **WHEN** a new POLY goal is prepared at any of the eight FOC divider phases
- **THEN** its first applied sample uses the existing reference time axis without an extra horizon or a divider/integral restart.

### Requirement: Velocity ramp

RAMP SHALL change reference velocity toward its target by at most servo_control_vel_ramp_rate × dt per step, without overshoot. Retargeting or a valid rate update SHALL preserve the active reference velocity. Once reached, the target SHALL be held exactly.

#### Scenario: Ramp reverses direction

- **WHEN** a positive reference receives a negative target
- **THEN** each reference increment is rate-limited through zero until the new target is reached.

### Requirement: Generator configuration validation

The five servo_control_* generator parameters SHALL default to zero; velocity_planning_tolerance SHALL default to 0.5. Finite values SHALL be nonnegative, infinities and negatives SHALL be rejected, and NaN SHALL restore the respective default. FILTER SHALL require positive bandwidth, RAMP positive rate, and POLY positive velocity/acceleration/deceleration limits. A live write making the active generator invalid SHALL be rejected without changing its active reference, target or accepted configuration.

#### Scenario: Disable a required parameter while active

- **WHEN** an active POLY mode receives a live zero acceleration-limit write
- **THEN** the write fails and the existing profile/settings remain active.
