# FOC current control

## Purpose

Define the common field-oriented current/voltage path and feedback estimate used beneath Servo, MIT and direct control modes.

## Requirements

### Requirement: Rotor prediction and measurement update

The rotor observer SHALL initialize from the current mechanical encoder angle with zero velocity and residual acceleration. Subsequent predictions SHALL use previous posterior position/velocity and constant total acceleration, then correct state with the shortest wrapped angle innovation. Electrical phase and shaft velocity used for the current control sample SHALL come from that sample's prior prediction. The observer SHALL maintain state per controller instance and assume prediction error within one half-turn for unambiguous wrapping.

#### Scenario: Encoder wrap

- **WHEN** successive rotor measurements cross the 2π/0 boundary while tracking continuously
- **THEN** the innovation uses the nearest angular branch rather than a full-turn error.

### Requirement: Current feedback and regulation

FOC SHALL transform measured phase currents into d/q using native electrical phase and low-pass filter them. Current-controlled modes SHALL use Id_ref=0 and separate current-error regulators for Id and Iq. Default current regulation SHALL be PI, with configured derivative support. It SHALL use the current-control period from the timing contract and the same electrical phase for forward current and inverse voltage transforms.

#### Scenario: Zero requested torque

- **WHEN** current-controlled torque demand is zero
- **THEN** Id_ref and the unclipped Iq_ref are zero; both axes still regulate measured current error.

### Requirement: Current reference clipping

For current-controlled modes, Iq_ref magnitude SHALL be limited by rated_max_torque/kt, the active positive runtime current limit, rated_max_current and MAX_BOARD_CURRENT=30 A. Runtime stall derating SHALL only reduce the available current. Stored user limits larger than ratings SHALL NOT raise effective ratings. Servo's torque-output saturation SHALL additionally enforce its effective user torque/current bounds. A reduced stored rating SHALL become an active rating when configuration is applied at restart.

#### Scenario: Rated current above the board cap

- **WHEN** active rated_max_current exceeds 30 A and a current-controlled mode asks for more than 30 A
- **THEN** the final Iq reference magnitude is at most 30 A even if user settings allow more.

### Requirement: Voltage and modulation limits

Current-regulator integration SHALL be gated by the explicit ±Vbus bounds, with configured output bounds also applying; the d-axis result SHALL additionally be clipped to ±Vbus. The d/q voltage vector SHALL then be magnitude-limited to Vbus. Inverse transformation and SVM SHALL produce duties clipped to [0,1], followed by the board PWM compare cap of 0.88. This vector bound SHALL NOT be interpreted as a guarantee that downstream modulation will avoid clipping.

#### Scenario: Requested voltage vector exceeds the bus bound

- **WHEN** sqrt(Vd²+Vq²) exceeds Vbus before vector limiting
- **THEN** its magnitude is reduced to Vbus while retaining its direction, before SVM and duty clipping.

### Requirement: Bus current estimate

Bus current SHALL be estimated in amperes as dA IA + dB IB + dC IC using measured phase currents and final applied duty ratios after SVM, integer PWM conversion and duty clipping. Duty ratios SHALL correspond to the sampled interval rather than the next control output. The signed estimate SHALL be low-pass filtered using i_lpf. It SHALL be zero for nonpositive bus voltage, OFF and bootstrap charging. Signed negative electrical power SHALL yield a negative estimate. This SHALL remain an estimate rather than a physical DC-link current measurement or current-control feedback.

#### Scenario: Consumption and regeneration

- **WHEN** Vbus=24 V, Vd=0, Vq=4 V and Iq=2 A in linear modulation at steady state
- **THEN** estimated bus current is approximately 0.5 A, allowing for PWM quantization
- **AND** reversing Iq with unchanged voltage yields approximately -0.5 A after the filter settles.

#### Scenario: Saturation uses applied duty

- **WHEN** Vbus=24 V, requested Vd=0, Vq=24 V, electrical angle=0 and Id=0, Iq=10 A produce final duty ratios 0.5, 0.88, 0
- **THEN** the steady-state bus-current estimate is approximately 7.62 A rather than the 15 A implied by the requested voltage.

#### Scenario: Next command does not alter sampled power

- **WHEN** a current sample is processed while preparing a different PWM output
- **THEN** the estimate uses the final duty ratios that applied during that sample, and the new PWM output contributes only to later samples.

### Requirement: Sensing during disabled operation

In RUNNING, OFF and bootstrap charging SHALL continue sensor/state updates without executing normal torque/current control. External shaft-sensor polling SHALL follow the control/sensor work once per nine update opportunities, including OFF/bootstrap. During bootstrap, zero compare values SHALL be used for the charging interval.

#### Scenario: OFF feedback

- **WHEN** the shaft moves externally while the RUNNING drive is OFF
- **THEN** angle/velocity sensing continues without turning bridge PWM back on.
