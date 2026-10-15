# Temperature protection

## Purpose

Protect VBDrive from stator overheating, MCU temperature excursions and invalid temperature measurements while retaining status reporting, configuration access and neutral recovery.

## Requirements

### Requirement: Stator thermal hysteresis

The stator interlock SHALL latch at T_stator >= max_temp and remain latched until a valid T_stator <= restart_temp. Comparisons SHALL use kelvins. The maximum threshold SHALL default to 363.15 K and the release threshold to 333.15 K. Cooling below max_temp alone SHALL NOT release a previously latched trip.

#### Scenario: Default heating and cooling cycle

- **WHEN** stator temperature follows 332.15, 333.15, 362.15, 363.15, 353.15, 333.15, 332.15 K with valid in-range MCU temperature
- **THEN** the stator interlock is clear, clear, clear, latched, latched, clear, clear respectively.

### Requirement: MCU operating range

The MCU interlock SHALL assert below 233.15 K or above 403.15 K and clear when valid MCU temperature returns to the inclusive range [233.15,403.15] K. These SHALL be named, fixed junction-temperature limits for the STM32G431VBx3 embedded in STSPIN32G4. Ambient, absolute-maximum and sensor-calibration values SHALL NOT substitute for operating junction limits.

#### Scenario: MCU recovers independently

- **WHEN** MCU temperature falls from 404.15 K to 403.15 K while stator temperature is valid and its overtemperature latch is clear
- **THEN** the MCU interlock clears without an additional MCU hysteresis threshold.

#### Scenario: MCU too cold

- **WHEN** MCU temperature is 232.15 K
- **THEN** protection inhibits the motor until valid temperature reaches at least 233.15 K.

### Requirement: Invalid sensor interlock

An unavailable, uninitialized, NaN/Inf or invalid-conversion reading from either sensor SHALL inhibit the bridge. Stator ADC endpoint values indicating a short/open circuit and invalid MCU reference/calibration inputs SHALL be reported as invalid instead of being accepted as a cold temperature. Invalid-reading inhibition SHALL clear only after both sensors are valid; any earlier stator overtemperature latch SHALL still require its restart threshold.

#### Scenario: Open stator sensor

- **WHEN** the stator conversion receives an ADC endpoint representing an open circuit
- **THEN** the motor is forced OFF even if the existing mathematical conversion would produce a finite low value.

#### Scenario: Validity returns above restart without prior overheating

- **WHEN** an invalid reading caused inhibition without a stator overtemperature latch and both sensors recover with T_stator=348.15 K and normal MCU temperature
- **THEN** invalid-reading inhibition clears, while thermal health is ADVISORY.

### Requirement: Protection status is orthogonal to command state

The software SHALL expose a distinct uint8-backed VBDriveStatus with explicit values NOMINAL=0 and COOLING_DOWN=1. COOLING_DOWN SHALL mean any active temperature/validity interlock, including MCU under-temperature; NOMINAL SHALL mean no such interlock. Temperature status changes SHALL NOT themselves replace CONFIG, RUNNING or another CommandState. In particular CONFIG and COOLING_DOWN SHALL coexist.

#### Scenario: Configuration during protection

- **WHEN** CONFIG is entered during cooling
- **THEN** command state is CONFIG, temperature status remains COOLING_DOWN, and the bridge stays OFF while queries and valid configuration writes remain available.

### Requirement: Forced OFF and controlled automatic recovery

Any active interlock SHALL force actual OFF with is_on=0, PWM main output disabled, previous control/reference/integral state cleared and the STSPIN logic supply retained. Once all interlocks clear, a drive that had enabled intent before the trip SHALL automatically enable at a neutral target when its command state permits. A new accepted movement command SHALL be required. A drive already deliberately OFF SHALL remain OFF. Explicit is_on=0 during protection SHALL cancel pending automatic recovery. A rejected is_on=1 during protection SHALL NOT create a new deferred enable request.

#### Scenario: Automatic neutral restart

- **WHEN** an enabled position-controlled drive trips at 363.15 K and cools to 333.15 K with valid MCU temperature
- **THEN** it automatically enables in RUNNING with a neutral target
- **AND** its preceding position goal, profile, integral and command index are not replayed.

#### Scenario: Explicit OFF wins

- **WHEN** is_on=0 is accepted during COOLING_DOWN and temperatures later recover
- **THEN** status can become NOMINAL but the bridge remains OFF until a new explicit enable succeeds.

#### Scenario: Cooling completes in CONFIG

- **WHEN** temperatures recover while command state is CONFIG and pre-trip enabled intent is pending
- **THEN** status becomes NOMINAL without energizing the bridge in CONFIG
- **AND** eligible enable restoration occurs only after returning to RUNNING, unless explicitly cancelled.

### Requirement: Protected entry and movement rejection

Startup enable, explicit enable, configuration restoration and calibration entry SHALL evaluate protection using initialized current measurements before energizing the bridge. Motion commands processed during COOLING_DOWN SHALL be rejected and SHALL NOT become pending targets. OFF and configuration access SHALL remain available. Once protection clears, previously pending motion SHALL NOT be replayed; recovery requires a fresh command accepted after neutral enable.

#### Scenario: Hot boot

- **WHEN** startup would normally enable a calibrated drive but initialized stator temperature is 368.15 K
- **THEN** it enters COOLING_DOWN without enabling PWM and may restore that startup enabled intent only after valid recovery.

### Requirement: Temperature servicing is independent of communications

Normal temperature conversion/evaluation SHALL be scheduled once per new millisecond in foreground, including while OFF or in CONFIG, independently of Cyphal publication or transport availability. The first service that detects an excursion SHALL force OFF in the same service pass. FOC/P/D SHALL retain their existing cadence; the thermal tracker SHALL NOT add temperature conversion to the 40 kHz path. Before enabling, a current measurement/evaluation SHALL prevent an old healthy sample from bypassing the interlock. Blocking calibration SHALL provide cooperative checks at intervals no longer than 10 ms while energized and abort on a trip.

#### Scenario: CAN unavailable

- **WHEN** Cyphal servicing is stopped but the drive continues ordinary foreground operation
- **THEN** temperature sampling, protection and recovery remain operational.

### Requirement: Heartbeat thermal health and status

Each scheduled Heartbeat SHALL publish vendor_specific_status_code equal to VBDriveStatus. Thermal health SHALL be CAUTION while any interlock is active; otherwise it SHALL be ADVISORY when T_stator >= restart_temp and NOMINAL when T_stator < restart_temp. The stator overtemperature latch SHALL keep CAUTION throughout cooling. Health and vendor status SHALL describe one coherent evaluation. The existing periodic Heartbeat cadence and CommandState/Cyphal mode responsibilities SHALL be retained.

#### Scenario: Status and health during a complete cycle

- **WHEN** valid stator temperature follows 332.15, 333.15, 363.15, 353.15, 333.15, 332.15 K with normal MCU temperature
- **THEN** successive health/status pairs are NOMINAL/0, ADVISORY/0, CAUTION/1, CAUTION/1, ADVISORY/0, NOMINAL/0.

#### Scenario: MCU recovery below stator advisory threshold

- **WHEN** an MCU range excursion clears and valid stator temperature is below restart_temp with no stator latch
- **THEN** the next scheduled Heartbeat reports NOMINAL/0; there is no artificial ADVISORY stage.
