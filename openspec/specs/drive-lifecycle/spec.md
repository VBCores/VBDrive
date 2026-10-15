# Drive lifecycle

## Purpose

Define application command states, bridge enable/disable, configuration sessions and isolated calibration without conflating command state with hardware enable.

## Requirements

### Requirement: Command state and bridge enable are separate

The application SHALL distinguish INIT, RUNNING, CONFIG, NOT_CALIBRATED and CALIBRATING. `is_on` SHALL report volatile bridge enable state independently of this command state. Motion command dispatch and log_on SHALL require RUNNING. `is_on=1` SHALL require RUNNING and an available motor; `is_on=0` SHALL be allowed whenever the motor is available. CALIBRATING SHALL isolate normal command processing.

#### Scenario: RUNNING while disabled

- **WHEN** a RUNNING drive receives is_on=0
- **THEN** the command state remains RUNNING while bridge PWM is disabled and normal command/register servicing remains available.

### Requirement: Startup and calibration loading

Startup SHALL load communication/application configuration before normal operation. Missing required configuration SHALL retain a setup path with Serial and Cyphal access; node_id=0 SHALL use a UID-derived temporary CAN node identity for setup. A configured startup SHALL initialize the motor with a neutral target and load calibration. Missing or incompatible calibration SHALL stop the bridge and enter NOT_CALIBRATED. A calibrated startup SHALL NOT restore a previous runtime motion command; is_on SHALL NOT be treated as persistent configuration or assumed to boot OFF.

#### Scenario: Calibration unavailable

- **WHEN** motor initialization finds a calibration record with the wrong type or no completed-calibration flag
- **THEN** the application enters NOT_CALIBRATED and leaves the bridge disabled.

### Requirement: Explicit OFF and neutral enable

OFF SHALL disable PWM main output, clear Servo history/reference state, reset current-controller integrators, set a neutral voltage target and clear the estimated bus current. It SHALL keep the STSPIN logic supply operating. In RUNNING, sensor updates SHALL continue while OFF. Enabling SHALL begin from neutral control with bootstrap charging; a new motion command SHALL be needed after enable. Repeated enable while already ON SHALL preserve the active target.

#### Scenario: OFF followed by ON

- **WHEN** a moving drive is disabled and later enabled successfully
- **THEN** the old command is not resumed, PWM starts through the bootstrap sequence and a fresh command is required.

#### Scenario: Logic supply remains active

- **WHEN** CONFIG or explicit OFF stops VBDrive powered from the STSPIN regulator
- **THEN** stopping PWM does not request whole-chip standby or intentionally remove the MCU supply.

### Requirement: STOP semantics

STOP SHALL clear Servo command/generator history and set a zero-voltage target while preserving the current is_on flag. It SHALL reply `STOP OK`. STOP SHALL NOT be specified as OFF, active position holding or a guaranteed mechanical brake.

#### Scenario: STOP on an enabled drive

- **WHEN** an enabled RUNNING drive processes STOP
- **THEN** its voltage target becomes zero and its bridge-enable flag remains enabled.

### Requirement: Configuration session lifecycle

CONFIG SHALL remember the preceding command state and enable flag, stop the motor and create a staged configuration. Repeated CONFIG SHALL NOT replace that original snapshot. EXIT SHALL discard staged edits, return to the preceding state and restore prior enable only when returning to RUNNING. Explicit OFF during CONFIG SHALL cancel that enable restoration. RESET SHALL stage defaults. SAVE SHALL persist the staged configuration, apply Servo settings and finish CONFIG in RUNNING, restoring remembered enable where applicable. APPLY SHALL stop the drive through the Serial service path, persist pending edits, acknowledge and reboot. These operations SHALL NOT replay the preceding movement target.

#### Scenario: Discard configuration without restoring motion

- **WHEN** CONFIG is entered from an enabled RUNNING drive, edits are staged and EXIT is sent
- **THEN** original committed values are restored and the drive can be re-enabled at a neutral target without replaying its prior trajectory.

#### Scenario: Explicit OFF cancels restoration

- **WHEN** is_on=0 is accepted during CONFIG and SAVE or EXIT follows
- **THEN** the remembered enable state does not cause automatic re-enabling.

### Requirement: Calibration isolation and completion

CALIBRATE SHALL be accepted from RUNNING or NOT_CALIBRATED, clear prior control, and enter CALIBRATING after enabling the motor if necessary. Serial input received during calibration SHALL be discarded, and Cyphal servicing SHALL be paused and reset before reuse of its shared memory. Progress SHALL use `CALIBRATE OK`, `CALIBRATE k/N DONE` after stages, and `CALIBRATE FINISH` after successful completion; a reported failure SHALL use `CALIBRATE ERROR: ...`. Successful completion SHALL store/apply calibration, restore RUNNING and communications, and return an initially OFF motor to OFF.

#### Scenario: Commands arrive during calibration

- **WHEN** normal movement lines arrive while calibration is executing
- **THEN** they are discarded rather than applied after calibration finishes.

### Requirement: Bootloader request lifecycle

Writing bootloader=1 SHALL latch a deferred bootloader request. Writing zero SHALL NOT cancel a pending request. Before the reboot into VBBoot, the application SHALL neutralize control and disable the bridge.

#### Scenario: Latched bootloader request

- **WHEN** bootloader=1 is followed by bootloader=0 before foreground dispatch
- **THEN** the request remains pending and dispatch stops the drive before reset.
