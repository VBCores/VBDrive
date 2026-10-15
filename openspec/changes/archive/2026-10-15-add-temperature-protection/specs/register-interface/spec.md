## MODIFIED Requirements

### Requirement: Shared typed register catalog

Serial and Cyphal SHALL expose the same 50 named registers with the types, mutability and persistence in the catalog below. Names SHALL remain case-sensitive, including the spelling `servo_control_input_bandwith`. Defaults SHALL resolve through the firmware parameter catalog rather than arbitrary example values. Read-only writes SHALL NOT change state. Runtime measurements SHALL report unavailable when their motor source does not exist; Cyphal SHALL return an empty value for an unavailable read.

#### Scenario: Equivalent reads and permissions

- **WHEN** the same committed parameter is read by Serial and Cyphal outside CONFIG
- **THEN** its typed value and units agree, and a read-only write on either path leaves it unchanged.

### Requirement: Parameter activation boundaries

Live Cyphal writes SHALL apply kt, user limits/direction/offset, Servo gains, generator settings and stator temperature thresholds when a motor is available and the candidate is valid. Other startup-configured parameters, including ratings, gear, communication baud/node identity, current gains, observer gains and encoder selection, SHALL take effect on restart. Serial SAVE SHALL apply Servo settings and valid stator temperature thresholds while other startup settings require APPLY/restart. Reading a stored value SHALL NOT imply that every startup-configured setting is already active. Serial baud SHALL default to 115200 and be supported by both application and VBBoot after restart.

#### Scenario: Live kt update

- **WHEN** a valid Cyphal kt write is accepted on a running motor
- **THEN** torque/current conversion uses the new constant without reboot and the configuration is marked for persistence.

#### Scenario: Rating update

- **WHEN** rated_max_current is written
- **THEN** the stored value changes and becomes the active motor rating after restart; a larger user limit cannot override that active rating.

#### Scenario: Lower the active stator threshold

- **WHEN** a valid live threshold write lowers max_temp below the measured stator temperature
- **THEN** the new protection policy is evaluated immediately in foreground without reboot, and the drive is forced OFF if it trips.


### Requirement: Persistence schemas and storage address

Application and VBBoot SHALL share the common configuration prefix and configured storage address, default zero. The common block SHALL keep type ID 0x01234567 independently of the application schema; application validation SHALL use its own type ID (0x44AAAC05). Invalid common/app blocks SHALL independently reset to their defaults. Configuration SHALL NOT overlap calibration at 0x0100 or sensor state at 0x2200, nor exceed EEPROM capacity. Changing the configured base address SHALL NOT implicitly relocate old data or migrate an incompatible application schema.

#### Scenario: App schema changes with valid communication prefix

- **WHEN** the common prefix is valid but the application type is incompatible
- **THEN** the common settings are retained and application data resets to defaults rather than being interpreted as the new layout.

#### Scenario: Upgrade from the previous application schema

- **WHEN** an EEPROM contains application type 0x44AAAC04 and a valid common prefix
- **THEN** the common configuration remains readable, the incompatible application configuration resets to defaults, and calibration/sensor regions are not overwritten
- **AND** fresh application configuration and calibration are required; old settings SHALL NOT be migrated or automatically restored. CalibrationData::TYPE_ID SHALL change from 0x89ABCDEF to 0x89ABCDF0, rejecting old calibration records without erasing their bytes.


### Requirement: Identity and telemetry meaning

`device` SHALL read `vbdrive`; `name` SHALL remain writable/persistent. Firmware and Cyphal software/VCS revision reporting SHALL derive from a version tag rather than a commit hash. bus_current SHALL follow the FOC estimate; temperature-measurement registers SHALL use kelvins and encoder registers raw counts. `is_fault` is currently a read-only placeholder and SHALL NOT be represented as a qualified hardware-fault indication.

#### Scenario: Device identity read

- **WHEN** a client changes name and reads device
- **THEN** device remains `vbdrive` and name returns the stored custom identity.

## ADDED Requirements

### Requirement: Stator protection registers

`max_temp` and `restart_temp` SHALL be read/write persistent real32 values in kelvins, with defaults 363.15 and 333.15 respectively. Their resolved pair SHALL satisfy 0 < restart_temp < max_temp with finite values. NaN SHALL restore the individual default; infinities and invalid pairs SHALL be rejected without disabling protection. Serial CONFIG SHALL permit individually valid staged edits and validate the final pair at SAVE/APPLY before persistence, runtime mutation or reboot. Each Cyphal write SHALL validate the complete candidate pair against committed values before accepting it. Successful live activation SHALL reevaluate existing protection latches against the new thresholds rather than resetting them.

| Register | Type | Access | Unit | Default |
| --- | --- | --- | --- | --- |
| `max_temp` | real32 | read/write, persistent | K | 363.15 |
| `restart_temp` | real32 | read/write, persistent | K | 333.15 |

#### Scenario: Reject invalid live pair

- **WHEN** committed max_temp=363.15 and a Cyphal write requests restart_temp=368.15
- **THEN** the request leaves committed values, active limits and EEPROM state unchanged.

#### Scenario: Staged pair is checked at commit

- **WHEN** Serial edits temporarily cross the threshold ordering but finish with a valid pair
- **THEN** SAVE can apply and persist that pair
- **AND** an invalid final pair returns a command-prefixed error and leaves CONFIG active without reboot or changing committed limits.

#### Scenario: Increasing max does not erase hysteresis

- **WHEN** the stator is latched hot at 353.15 K, restart_temp is 333.15 K and max_temp is increased from 363.15 K to 383.15 K
- **THEN** the stator latch remains active until temperature reaches restart_temp or that release threshold is validly changed.
