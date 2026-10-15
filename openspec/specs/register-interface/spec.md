# Serial and Cyphal register interface

## Purpose

Define the shared register catalog, transport command formats, configuration persistence and compatibility visible to VBDrive clients.

## Requirements

### Requirement: Shared typed register catalog

Serial and Cyphal SHALL expose the same 50 named registers with the types, mutability and persistence in the catalog below. Names SHALL remain case-sensitive, including the spelling `servo_control_input_bandwith`. Defaults SHALL resolve through the firmware parameter catalog rather than arbitrary example values. Read-only writes SHALL NOT change state. Runtime measurements SHALL report unavailable when their motor source does not exist; Cyphal SHALL return an empty value for an unavailable read.

#### Scenario: Equivalent reads and permissions

- **WHEN** the same committed parameter is read by Serial and Cyphal outside CONFIG
- **THEN** its typed value and units agree, and a read-only write on either path leaves it unchanged.

### Requirement: Serial framing and status

Serial SHALL accept CR, LF or CRLF-delimited commands, `<name>:?` reads and `<name>:<value>` writes. Reads SHALL return `<name>:<value>`. Status replies SHALL start with the command/register name, including `servo_cmd OK`, `mit_cmd OK` and `<name> ERROR: ...`; a bare status or status-first reply SHALL NOT substitute for this form. Overlong/overflowed input SHALL be discarded through a command boundary and reported as an input error, without executing a truncated target.

#### Scenario: Invalid movement line

- **WHEN** an incomplete or malformed servo_cmd line is processed in RUNNING
- **THEN** it returns `servo_cmd ERROR: ...`, increments the movement-error counter once and preserves the active target.

### Requirement: Motion wire formats

Serial SHALL accept `servo_cmd: <type> <value> [command_idx]` with an optional integer index 0..255 and `mit_cmd: <pos> <vel> <torq> <p_gain> <v_gain>`. Cyphal SHALL subscribe to Servo at 3407+node_id and MIT at 2107+node_id, and publish State at subject 3811. Servo SHALL use uint8 control_type, float32 set_point_value and a uint8 array of length zero or one for command_idx. MIT SHALL accept the 20-byte DSDL payload and the supported 28-byte form with the same prefix, ignoring its two trailing current-gain fields. State SHALL carry a microsecond timestamp and output position, velocity and torque. Both transports SHALL dispatch accepted motion commands through the same semantics.

#### Scenario: Omitted index

- **WHEN** a Serial Servo line omits command_idx or its Cyphal array is empty
- **THEN** both use the unindexed repeat rules in the control-modes specification.

### Requirement: Staged Serial and committed Cyphal configuration

Persistent Serial writes SHALL require CONFIG and modify staged values; Serial queries in CONFIG SHALL read those staged values. Cyphal persistent access SHALL operate on the committed configuration, with successful writes scheduled for deferred EEPROM saving. EXIT SHALL discard only the staged Serial transaction. If SAVE commits a complete staged snapshot, that snapshot SHALL become committed even when Cyphal writes occurred during editing; field-level merging SHALL NOT be assumed. Runtime controls SHALL remain nonpersistent.

#### Scenario: Cyphal write during Serial editing

- **WHEN** a committed parameter is changed by Cyphal while a Serial CONFIG snapshot is open and EXIT follows
- **THEN** the committed Cyphal change survives while staged Serial edits are discarded.

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

### Requirement: Value validation and reset semantics

kt and both ratings SHALL resolve NaN to defaults (1 N m/A and 30 N m/30 A) and SHALL reject nonpositive or non-finite resolved values. Servo PID gains SHALL be finite and nonnegative. Generator parameters SHALL follow trajectory-generation validation. ang_dir SHALL accept exactly ±1. `gear` SHALL accept integers 1..255; configured node IDs SHALL be 1..127, with zero reserved as the unset default. `name` SHALL be 1..15 bytes and exclude control bytes. User-limit NaN values SHALL remove the corresponding user restriction without lifting rated/board bounds.

#### Scenario: Invalid kt and mutable name

- **WHEN** kt=0 is requested
- **THEN** it is rejected without changing active or stored kt
- **AND** a valid name write changes writable identity while the read-only device value remains `vbdrive`.

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

## Register catalog

`RW/P` denotes read/write persistent configuration, `RW` a runtime control, and
`RO` nonpersistent information. `—` means no persistent default. Float literals
below are the effective defaults in the current firmware catalog; stored NaN can
represent an unresolved default. A default gear read of 36 does not provision the
required stored gear field; gear and node identity must be explicitly configured.

| Name | Type | Access | Default |
| --- | --- | --- | --- |
| `gear` | natural32 | RW/P | `36` |
| `max_i` | real32 | RW/P | `NaN` |
| `max_spd` | real32 | RW/P | `NaN` |
| `max_tq` | real32 | RW/P | `NaN` |
| `ang_off` | real32 | RW/P | `0.0` |
| `ang_dir` | integer32 | RW/P | `1` |
| `min_ang` | real32 | RW/P | `NaN` |
| `max_ang` | real32 | RW/P | `NaN` |
| `kt` | real32 | RW/P | `1.0` |
| `kp` | real32 | RW/P | `4.0` |
| `ki` | real32 | RW/P | `1600.0` |
| `kd` | real32 | RW/P | `0.0` |
| `flt_a` | real32 | RW/P | `0.0` |
| `flt_g1` | real32 | RW/P | `0.033554277` |
| `flt_g2` | real32 | RW/P | `18.974049687` |
| `flt_g3` | real32 | RW/P | `4444.260353864` |
| `i_lpf` | real32 | RW/P | `0.0925` |
| `ang_enc` | natural32 | RW/P | `0` |
| `node_id` | natural32 | RW/P | `0` |
| `data_baud` | natural32 | RW/P | `3` |
| `nominal_baud` | natural32 | RW/P | `4` |
| `is_on` | bit | RW | `—` |
| `bootloader` | bit | RW | `—` |
| `cmd_errors` | natural32 | RO | `—` |
| `name` | string | RW/P | `vbdrive` |
| `firmware_rev` | string | RO | `—` |
| `bus_voltage` | real32 | RO | `—` |
| `bus_current` | real32 | RO | `—` |
| `temp_mcu` | real32 | RO | `—` |
| `temp_stator` | real32 | RO | `—` |
| `is_fault` | bit | RO | `—` |
| `encoder_shaft` | natural32 | RO | `—` |
| `encoder_rotor` | natural32 | RO | `—` |
| `servo_pos_p_gain` | real32 | RW/P | `150.0` |
| `servo_pos_i_gain` | real32 | RW/P | `200.0` |
| `servo_pos_d_gain` | real32 | RW/P | `10.0` |
| `servo_vel_p_gain` | real32 | RW/P | `30.0` |
| `servo_vel_i_gain` | real32 | RW/P | `60.0` |
| `servo_control_input_bandwith` | real32 | RW/P | `0.0` |
| `servo_control_vel_limit` | real32 | RW/P | `0.0` |
| `servo_control_accel_limit` | real32 | RW/P | `0.0` |
| `servo_control_decel_limit` | real32 | RW/P | `0.0` |
| `servo_control_vel_ramp_rate` | real32 | RW/P | `0.0` |
| `serial_baud` | natural32 | RW/P | `115200` |
| `device` | string | RO | `—` |
| `rated_max_torque` | real32 | RW/P | `30.0` |
| `rated_max_current` | real32 | RW/P | `30.0` |
| `velocity_planning_tolerance` | real32 | RW/P | `0.5` |
| `max_temp` | real32 | RW/P | `363.15` |
| `restart_temp` | real32 | RW/P | `333.15` |
