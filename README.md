# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## Configuration and Control Registers (Serial and Cyphal)

All 44 registers are shared. The following configuration registers are read/write
and persistent; runtime controls `is_on` and `bootloader` are marked separately.

| Parameter | Description | Type | Default |
| --- | --- | --- | --- |
| `gear` | Gear ratio of the drive | Integer | `36` |
| `name` | Writable device name (1-15 bytes) | String | `vbdrive` |
| `max_i` | Maximum motor current (A) | Float | `NaN` |
| `max_spd` | Maximum motor speed target (rad/s) | Float | `NaN` |
| `max_tq` | Maximum torque output (Nm) | Float | `NaN` |
| `ang_off` | Joint angle offset (rad) | Float | `0.0` |
| `ang_dir` | Joint angle direction multiplier, -1 or +1 | Integer | `1` |
| `min_ang` | Minimum allowed angle (rad) | Float | `NaN` |
| `max_ang` | Maximum allowed angle (rad) | Float | `NaN` |
| `kt` | Torque constant (Nm/A) | Float | `1.0` |
| `kp` | Current proportional gain | Float | `4.0` |
| `ki` | Current integral gain | Float | `1600.0` |
| `kd` | Current derivative gain | Float | `0.0` |
| `flt_a` | Main filter parameter A | Float | `0.0` |
| `flt_g1` | Filter gain 1 | Float | `0.015700989410003974` |
| `flt_g2` | Filter gain 2 | Float | `3.925227776360174` |
| `flt_g3` | Filter gain 3 | Float | `387.54711795263574` |
| `i_lpf` | Current low-pass filter coefficient | Float | `0.0925` |
| `ang_enc` | Angle encoder type enum, 0 rotor, 1 shaft | Integer | `0` (rotor) |
| `node_id` | Cyphal/CAN node ID | Integer | `0` (unset) |
| `data_baud` | FDCAN data baud rate enum (see below) | Enum | `3` (8 MHz) |
| `nominal_baud` | FDCAN nominal baud rate enum (see below) | Enum | `4` (1 MHz) |
| `serial_baud` | Serial baud rate, applied after reboot | natural32 | `115200` |

To clear a user-configured limit, write `NaN` to the corresponding register:
`min_ang`, `max_ang`, `max_spd`, `max_i`, or `max_tq`. Each limit is
independent. `NaN` removes the angle bound or commanded-speed bound; it does
not permit a `NaN` movement target. For `max_i` and `max_tq`, `NaN` removes
only the user limit: the hardware limits (30 A and 30 Nm) and stall derating
still apply. `max_spd` checks velocity targets; it does not cap measured speed
in POSITION mode. Over Serial, send a value such as `min_ang:nan` in
`CONFIG`, then use `APPLY` to save and reboot. Cyphal writes to these limit
registers apply at runtime and are saved.

`node_id = 0` is an unset configuration value. An unconfigured device uses a
temporary setup node ID derived from its MCU UID.

| Runtime control | Type | Access | Persistent | Meaning |
| --- | --- | --- | --- | --- |
| `is_on` | bit | read/write | no | Enable driver; Serial accepts 0/1 in RUNNING |
| `bootloader` | bit | read/write | no | 1 requests VBBoot; 0 is a no-op, not cancellation |

### Servo Parameters

Servo parameters are shared Serial/Cyphal read/write persistent registers:

| Name | Type | Default |
| --- | --- | --- |
| `servo_pos_p_gain` | real32 | 150 |
| `servo_pos_i_gain` | real32 | 200 |
| `servo_pos_d_gain` | real32 | 10 |
| `servo_vel_p_gain` | real32 | 30 |
| `servo_vel_i_gain` | real32 | 60 |
| `servo_control_input_bandwith` | real32 | 0 |
| `servo_control_vel_limit` | real32 | 0 |
| `servo_control_accel_limit` | real32 | 0 |
| `servo_control_decel_limit` | real32 | 0 |
| `servo_control_vel_ramp_rate` | real32 | 0 |

Gains must be finite and non-negative. Generator parameters accept finite
non-negative values; `NaN` restores the default zero. A generator command
requires all of its parameters to be positive, otherwise it is rejected without
changing the active target. Filter bandwidth is in 1/s; velocity limits and
ramp rate are in output rad/s and rad/s². POSITION uses
`motor_torque = Kp * (target - position) + I - Kd * velocity`; VELOCITY uses
`motor_torque = Kp * (target - velocity) + I`. These are independent controllers,
P/D run every 25 microseconds. I accumulates those error samples and updates
once per five ticks, in motor-side N m.
Torque is limited by hardware, user output-shaft torque (converted through
`gear`) and available current (including stall derating), with conditional
integration to prevent windup. Position D uses measured velocity,
so target steps do not cause derivative kick. `max_spd` validates VELOCITY targets;
it does not limit actual speed in POSITION. Zero gains produce zero torque.
With `Ki = 0`, MIT feedforward torque and desired velocity both zero, matching
POSITION `Kp` and `Kd` values request the same motor current in MIT and SERVO.
For VELOCITY, matching `Kp` values and `Ki = 0` likewise give the same current
as MIT velocity control with zero position gain and feedforward.
`POSITION_FILTER` smooths the position input with a critically damped second-order
filter. Its effective bandwidth is capped at one quarter of the 5 kHz reference
update rate (1250 s^-1). `POSITION_POLY` plans a trapezoidal velocity profile from the measured
position and velocity to the goal with zero final velocity; short moves have a
triangular velocity profile. `VELOCITY_RAMP` limits the change of velocity input
per second. These modes shape the input of the existing Servo PID; its feedback
does not alter the generated reference. Limits govern the reference, not actual
motor motion. Configuration changes take effect immediately.

Servo control IDs are 0 VELOCITY_DIRECT, 1 VELOCITY_RAMP, 2 TORQUE_DIRECT,
3 POSITION_DIRECT, 4 POSITION_FILTER, 5 POSITION_POLY, 6 VOLTAGE_DIRECT.
The Servo wire format contains `control_type`, `set_point_value` and an
optional one-byte `command_idx`.
An absent index deduplicates consecutive identical commands. With an index,
repeating the index with changed type/value is invalid and increments
`cmd_errors`; a changed index starts a new command. Only POSITION_POLY replans
from measurements when a new command arrives. FILTER and RAMP continue their
reference; STOP, disable, MIT and calibration reset command history.
At high command rates the three-frame FDCAN receive FIFO uses overwrite mode,
so a full FIFO retains the newest frames. The firmware keeps no second command queue.

These starting gains were smoke-tested on M4310 with gear=36, kt=0.5,
max_i=0.3 A and max_tq=5 Nm, using small commands in both directions.
They are not load-independent tuning; check them with the actual mechanics and
current/torque limits. Defaults apply to fresh configuration and RESET. The
new EEPROM type requires restoring saved parameters and recalibrating once.

`ParameterDefinition::default_value` in `App/config/config.hpp` holds the effective defaults. Float fields in `VBDriveConfig`
are `NAN` until explicitly set. Generator parameters read as zero by default.
Serial/Cyphal reads and motor initialization resolve these sentinels identically.
A default does not mark a required parameter as configured: stored `gear=0`
still means unset even though its effective default is 36. Measurements have no default.
Servo uses two libvoltbro `PIDRegulator` instances, with explicit measured
derivative and conditional integration; FOC holds no separate PID state.

Serial writes require CONFIG: SAVE applies Servo gains, APPLY saves and reboots,
and EXIT discards staged changes. Other settings may still require APPLY.
Cyphal gain writes apply before the next FOC tick and are saved by the existing
deferred-save path, without exposing unsaved Serial CONFIG values.
Changing a gain resets that controller's state but retains its target; rewriting
the same gain or changing a target within the same mode does not reset it.
Changing control mode resets both Servo integrators. Disable/enable clears the old
target and waits at zero effort for a new command. TORQUE, VOLTAGE and MIT retain
their control laws. Register identifiers are string views,
not heap-allocated strings. The composite `servo_params` register is not used.

Configuration is stored as two contiguous, independently identified blocks:

| EEPROM offset | Bytes | Contents | Type ID |
| --- | ---: | --- | --- |
| `VB_CONFIG_ADDRESS` (default `0x0000`) | 28 | Common `BaseConfigData`: node ID, CAN/Serial rates, name, configured flag | `0x01234567` |
| `VB_CONFIG_ADDRESS + 0x1C` | 110 | `VBDriveConfig`: motor, limits, observer and Servo settings | `0x44AAAC02` |
| `0x0100` | 8204 | Motor calibration | `0x89ABCDEF` |
| `0x2200` | 8 | Inductive encoder setup state | `0xAAAAAA99` |

`-DVB_CONFIG_ADDRESS=0` selects the configuration byte address for both the
application and the VBBoot sub-build. Decimal and hexadecimal values are accepted.
The complete record must fit the 32 KiB EEPROM without overlapping calibration
or encoder state. Those two regions keep their fixed addresses. A different
address selects a different record; existing bytes are not moved.

The application saves the 138-byte configuration together. A later change of
application schema resets only its block; common communication settings and name
survive. Calibration has a fixed address independent of configuration size.
VBBoot reads the common block from the same C-compatible libvoltbro header.

Schema mismatches reset the corresponding configuration block. Configuration
records are not automatically migrated or relocated.
Flashing MCU firmware alone does not erase external EEPROM.


---

## Readonly Information Registers (Serial and Cyphal)

All entries are non-persistent and readable without CONFIG. Writes are rejected.

| Name | Type | Meaning |
| --- | --- | --- |
| `cmd_errors` | natural32 | Rejected Serial/Cyphal movement commands |
| `firmware_rev` | string | 16 hexadecimal digits of the VBDrive HEAD commit |
| `bus_voltage` | real32 | Bus voltage, V |
| `bus_current` | real32 | Working-current measurement, A; not a separate DC-link sensor |
| `temp_mcu` | real32 | MCU temperature, K |
| `temp_stator` | real32 | Stator temperature, K |

Both temperature registers report kelvins: subtract 273.15 to get degrees Celsius.
`temp_mcu` is the temperature of the embedded STM32G431 die in the central part
of STSPIN32G4, not the temperature of its package surface or ambient air. `temp_stator`
uses the external THERM1 divider;
its value is the motor sensor temperature.
| `is_fault` | bit | Reports false; DRV_FAULT integration remains deferred |
| `encoder_shaft` | natural32 | Raw external encoder counts |
| `encoder_rotor` | natural32 | Raw internal encoder counts |

Before motor initialization, unavailable measurements return a Serial error or an
empty Cyphal value. `firmware_rev` also supplies GetInfo's VCS revision; it identifies
the commit, not uncommitted changes.

### **FDCAN Baud Rate Configuration**

| parameter           | Value Name | Speed    | Numeric Value |
|---------------------|------------|----------|---------------|
| `nominal_baud`            | `KHz62`    | 62.5 kHz | `0`           |
|                     | `KHz125`   | 125 kHz  | `1`           |
|                     | `KHz250`   | 250 kHz  | `2`           |
|                     | `KHz500`   | 500 kHz  | `3`           |
|                     | `KHz1000`  | 1 MHz    | `4`           |
|---------------------|------------|----------|---------------|
| `data_baud`            | `KHz1000`  | 1 MHz    | `0`           |
|                     | `KHz2000`  | 2 MHz    | `1`           |
|                     | `KHz4000`  | 4 MHz    | `2`           |
|                     | `KHz8000`  | 8 MHz    | `3`           |

`nominal_baud` and `data_baud` are available through both Serial and Cyphal. Writes update EEPROM configuration; active CAN timing changes only after reboot.

---

## UART Configuration / Test Interface

The board uses a UART-based serial interface for configuration, calibration, motor commands, and state logging.

### **Connection Details**

* **Baud Rate**: `serial_baud`, default 115200 (application and VBBoot)
* **Format**: ASCII commands; trailing `\r`, `\n`, spaces and tabs are stripped automatically

Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400,
460800, 921600 and 1000000. `SAVE` persists a baud change without changing the
active connection; `APPLY` or a restart activates it. Reconnect at the new rate
after reboot. An invalid common block uses 115200.

---

### **Command Syntax**

* **Read parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` -> `node_id:11`. Queries work without `CONFIG`, except during calibration.

* **Write parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` -> `kp:0.350000 OK`

---

### Commands

Every command requires CR, LF or CRLF. A UART idle event is not a command terminator.
The maximum line is 191 bytes excluding its terminator. Commands may span UART
packets; several lines may arrive together. Invalid/overflowed lines are discarded
through a delimiter and produce an error, never a partial motion command.

| Command | State | Meaning |
| --- | --- | --- |
| `CONFIG` | Except CALIBRATING | Stop motor and stage configuration |
| `INFO` | Except CALIBRATING | Repeat startup information with configured settings (including staged values in CONFIG) |
| `HELP` | Except CALIBRATING | List Serial commands and syntax |
| `EXIT` | CONFIG | Discard staged changes, including RESET, and restore previous state |
| `SAVE` | CONFIG | Persist changes, apply Servo gains, exit CONFIG |
| `RESET` | CONFIG | Stage fresh defaults; EXIT discards them |
| `APPLY` | Except CALIBRATING | Save pending changes and reboot |
| `CALIBRATE` | RUNNING, NOT_CALIBRATED | Run isolated blocking calibration |
| `STOP` | Except CALIBRATING | Set voltage target to 0 |
| `mit_cmd: <pos> <vel> <torq> <p_gain> <v_gain>` | RUNNING | Apply the same MIT command as Cyphal |
| `servo_cmd: <type> <value> [command_idx]` | RUNNING | Types 0–6 as listed above; index 0–255 is optional |
| `log_on` | RUNNING | Enable state log at 10 ms intervals (100 Hz) |
| `log_off` | Any | Disable state log |

MIT arguments and Servo values are finite numbers separated by spaces/tabs.
Exactly the specified argument count is required. Invalid movement leaves the
previous target unchanged and increments `cmd_errors` once.
Movement commands do not enable a disabled driver. STOP outside calibration
does not change driver enable, CONFIG contents or logging. Use `is_on:0` to disable.

TEST, do_vel, do_ang, do_free and BOOT are removed. Use `bootloader:1` instead of
BOOT; it schedules a reboot without saving staged settings. `bootloader:0` does
not cancel an accepted request. The bool register is identical in Cyphal.
Calibration is an isolated blocking procedure: no commands, including STOP,
queries or bootloader requests, are processed until it finishes. Serial input
received during calibration is discarded, not executed afterwards. Cyphal is
paused, its TX/RX allocations are released, and its arena is temporarily used by
calibration. The arena and transport resume before FINISH without an MCU reset;
frames pending at the calibration boundary are discarded.
`CALIBRATE OK` acknowledges acceptance. `CALIBRATE 1/10 DONE` through
`CALIBRATE 10/10 DONE` are emitted after the ten movement stages;
`CALIBRATE FINISH` follows successful EEPROM save and
application of the result. Wait for FINISH before sending another command.
Calibration cannot be cancelled. If the driver is off, CALIBRATE enables it
for the calibration motion and disables it again after completion.

Logging is disabled when leaving RUNNING and does not restart automatically.
Its format uses the position, velocity and torque fields of `voltbro.foc.State`,
in rad, rad/s and N m, without timestamp:

```text
state: 0.000000 0.000000 0.000000
```

Status replies start with the command or register, then the status: `mit_cmd OK`,
`servo_cmd OK`, `STOP OK`, `<register>:<value> OK`, or `<command> ERROR: reason`.
Read replies remain `<register>:<value>`.
All Serial exchanges use the same shared register catalog.

```text
CONFIG
servo_pos_p_gain:150
servo_pos_i_gain:200
servo_pos_d_gain:10
SAVE
is_on:1
log_on
servo_cmd: 0 0.05
STOP
log_off
is_on:0
cmd_errors:?
```

## FDCAN Cyphal Runtime Interface

The BLDC Motor Controller communicates over **Cyphal/FDCAN** to publish real-time telemetry and receive control commands.

> We use some custom datatypes, see here: [VoltBro cyphal types repository](https://github.com/voltbro/cyphal-types)

The MIT subscriber accepts the 20-byte `voltbro.foc.MIT.1.0` payload and a
28-byte payload with the same prefix. The two trailing current-gain fields in
the 28-byte form are ignored; current regulator gains come from configuration.


### **Published Messages**

| Port ID | Message Type                              | Interval | Description                                   |
| ------- | ----------------------------------------- | -------- | --------------------------------------------- |
| `3811`  | `voltbro.foc.State.1.0`                   | 1 ms     | Timestamp, position, velocity, torque         |

---

### **Subscribed Messages**


| Port ID Formula  | Message Type                       | Description                                                                 |
| ---------------- | ---------------------------------- | --------------------------------------------------------------------------- |
| `2107 + node_id` | `voltbro.foc.MIT.1.0` | Torque, position, velocity, position gain and velocity gain |
| `3407 + node_id` | `voltbro.foc.Servo.1.0` | Types 0–6; uint8 control_type, float32 set_point_value, optional uint8 command_idx |


---

All registers are listed in the shared configuration/control and readonly tables
above. Cyphal persistent writes use the existing deferred EEPROM-save path.
An unconfigured device starts Cyphal in maintenance mode with a deterministic
setup node ID derived from the MCU UID.

### **Angle Frame Semantics**

> NOTE: angle offset only makes sense with ang_enc=1 (shaft output encoder). Rotor encoder (0) is not absolute in reference to shaft position

Control positions and velocities over Serial and Cyphal use the corrected joint frame:

* `reported_angle = measured_shaft_angle * ang_dir + ang_off`
* `voltbro.foc.MIT.pos`/`vel`, `voltbro.foc.Servo` position/velocity targets, `min_ang`, and `max_ang` are all interpreted in that corrected frame
* MIT position, velocity and feedforward torque terms are converted together to the physical motor direction
* Positive `ang_off` increases the reported and commanded joint angle for the same physical shaft position
* Units are radians

This means limit enforcement and position control are applied after the offset is added, so the configured limits match the angles seen by higher-level kinematics.
Over Serial, set `ang_dir` in CONFIG and run APPLY to make it active.
Cyphal register writes apply at runtime and persist to EEPROM.

### **Calibration Workflow**

1. Move the joint to the desired mechanical zero.
2. Read the measured joint angle.
3. Compute the required offset so the reported angle becomes zero:
   `ang_off = -measured_shaft_angle * ang_dir`
4. Write `ang_off` via `uavcan.register.Access`.
5. Read back `ang_off`, `min_ang`, and `max_ang` to confirm the corrected frame.
6. Set `min_ang` and `max_ang` in the same corrected frame.

---

### **Standard Cyphal Services**

The controller also supports standard Cyphal services:

* **uavcan.node.GetInfo** — Reports node information (name: `"org.voltbro.vbdrive"`)
* **uavcan.register.Access** — For reading/writing "high-level" parameters
* **uavcan.register.List** — List registers
* **uavcan.node.Heartbeat** — Node status monitoring

### **Standard Cyphal Messages**

The controller also publishes standard Cyphal messages:

* **uavcan.node.Heartbeat** - default heartbeat message

---

## Build and verification

Use `Release` for normal operation. `Debug` retains full symbols and MONITOR hooks,
with application `-O3` and support libraries/bootloader `-Os`; optimized local variables may be unavailable in the debugger. Failed assertions disable PWM, retain `assertion_file`, `assertion_line` and `assertion_expression` for the debugger, and enter the fatal handler. Initialize submodules before configuring. DSDL C headers and C++ traits are generated into the build directory using the CMake module and templates supplied by libcxxcanard. The firmware includes the generated build-directory types.

`VBDrive_full.hex` combines VBBoot at `0x08000000` with VBDrive at `0x08003000`.
The bootloader-compatible common block uses fixed type `0x01234567`.
The application and VBBoot use the same common configuration definition;
application-specific schema validation belongs to the application.


### Control scheduling

FOC current regulation and Servo P/D run at 40 kHz. Servo input generators
advance at 5 kHz with `dt = 200 us`; the next reference is held for eight FOC
ticks. Servo integral accumulates all five error samples and commits at 8 kHz
(`125 us` per update), with anti-windup and per-tick output limiting.
Cyphal receives one frame per main-loop pass and publishes fresh State at 1 kHz;
overdue State samples are not replayed. Serial parsing also runs in the main
loop, at most one command per millisecond. EEPROM saves, APPLY, calibration and
restarts are maintenance operations outside the steady-state timing guarantee.
