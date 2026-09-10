# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## Configuration and Control Registers (Serial and Cyphal)

All 40 registers are shared. The following configuration registers are read/write
and persistent; runtime controls `is_on` and `bootloader` are marked separately.

| Parameter  | Description                                               | Type    | Example Values |
| ---------- | --------------------------------------------------------- | ------- | -------------- |
| `gear`     | Gear ratio of the drive                                   | Integer | `1`, `5`, `15` |
| `max_i`    | Maximum motor current (A)                                 | Float   | `5.0`, `10.5`  |
| `max_spd`  | Maximum motor speed (rad)                                 | Float   | `1000.0`       |
| `max_tq`   | Maximum torque output (Nm)                                | Float   | `1.2`          |
| `ang_off`  | Joint angle offset (rad)                                  | Float   | `0.0`, `15.5`  |
| `ang_dir`  | Joint angle direction multiplier, -1 or +1 | Integer | `1`, `-1` |
| `min_ang`  | Minimum allowed angle (rad)                               | Float   | `-30.0`        |
| `max_ang`  | Maximum allowed angle (rad)                               | Float   | `30.0`         |
| `kt`       | Torque constant (Nm/A)                                    | Float   | `0.12`         |
| `kp`       | Current proportional gain                                 | Float   | `0.25`         |
| `ki`       | Current integral gain                                     | Float   | `0.01`         |
| `kd`       | Current derivative gain                                   | Float   | `0.005`        |
| `flt_a`    | Main filter parameter A                                   | Float   | `0.5`          |
| `flt_g1`   | Filter gain 1                                             | Float   | `0.1`          |
| `flt_g2`   | Filter gain 2                                             | Float   | `0.1`          |
| `flt_g3`   | Filter gain 3                                             | Float   | `0.1`          |
| `i_lpf`    | Current low-pass filter coefficient                       | Float   | `0.1`          |
| `ang_enc`  | Angle encoder type enum, 0 rotor, 1 - shaft               | Integer | `0`, `1`.      |
| `node_id`  | Cyphal/CAN node ID                                        | Integer | `1`, `42`      |
| `data_baud`   | FDCAN data baud rate enum (see below) | Enum    | `0`, `1`, `2`  |
| `nominal_baud`   | FDCAN nominal baud rate enum (see below) | Enum | `3`, `4`       |

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
| `servo_tr_form` | natural32 | 1 (`LINE_TRAJ`; 2 = `POLYNOM_TRAJ`) |
| `servo_tr_vel` | real32 | 0 |

Gains and transient velocity must be finite and non-negative. POSITION uses
`torque = Kp * (target - position) + I - Kd * velocity`; VELOCITY uses
`torque = Kp * (target - velocity) + I`. These are independent controllers, running
every 25 microseconds, with `I += Ki * error * dt` in output-shaft N m. Torque is
limited by hardware, user torque and available current (including stall derating),
with conditional integration to prevent windup. Position D uses measured velocity,
so target steps do not cause derivative kick. `max_spd` validates VELOCITY targets;
it does not limit actual speed in POSITION. Zero gains produce zero torque.
`servo_tr_form` and `servo_tr_vel` are stored but do not generate trajectories.
The wire format matches legacy `specific_control` for modes 0–3; other modes are rejected.

These starting gains were smoke-tested on M4310 with gear=36, kt=0.5,
max_i=0.3 A and max_tq=5 Nm, using small commands in both directions.
They are not load-independent tuning; check them with the actual mechanics and
current/torque limits. Defaults apply to fresh configuration and RESET; existing
EEPROM values are retained when flashing.

`VBDriveDefaults` holds the effective defaults. Float fields in `VBDriveConfig`
are `NAN` until explicitly set; integer `servo_tr_form` uses 0 as its unset value.
Serial/Cyphal reads and motor initialization resolve these sentinels identically.
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
their control laws. Names are string views (up to 16 characters),
not heap-allocated strings. The composite `servo_params` register is not used.

All settings, including Servo, are stored in one 102-byte config at EEPROM offset 0.
Calibration starts at 103, followed by encoder state. There is no old-layout migration.
Provision devices with erased external EEPROM, then configure and calibrate afresh;
flashing MCU firmware alone does not erase external EEPROM.


---

## Readonly Information Registers (Serial and Cyphal)

All entries are non-persistent and readable without CONFIG. Writes are rejected.

| Name | Type | Meaning |
| --- | --- | --- |
| `cmd_errors` | natural32 | Rejected Serial/Cyphal movement commands |
| `vbdrive_model` | string | CMake constant M4310 |
| `firmware_rev` | string | 16 hexadecimal digits of the VBDrive HEAD commit |
| `bus_voltage` | real32 | Bus voltage, V |
| `bus_current` | real32 | Working-current measurement, A; not a separate DC-link sensor |
| `temp_mcu` | real32 | MCU temperature, K |
| `temp_stator` | real32 | Stator temperature, K |
| `is_fault` | bit | Currently false; DRV_FAULT remains deferred |
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

* **Baud Rate**: 115200 (application and VBBoot)
* **Format**: ASCII commands; trailing `\r`, `\n`, spaces and tabs are stripped automatically

---

### **Command Syntax**

* **Read parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` -> `node_id:11`. Queries work without `CONFIG`, except during calibration.

* **Write parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` -> `OK: kp:0.350000`

---

### Commands

Every command requires CR, LF or CRLF. A UART idle event is not a command terminator.
The maximum line is 191 bytes excluding its terminator. Commands may span UART
packets; several lines may arrive together. Invalid/overflowed lines are discarded
through a delimiter and produce an error, never a partial motion command.

| Command | State | Meaning |
| --- | --- | --- |
| `CONFIG` | Except CALIBRATING | Stop motor and stage configuration |
| `INFO` | Except CALIBRATING | Repeat startup information with current (staged in CONFIG) settings |
| `HELP` | Except CALIBRATING | List Serial commands and syntax |
| `EXIT` | CONFIG | Discard staged changes, including RESET, and restore previous state |
| `SAVE` | CONFIG | Persist changes, apply Servo gains, exit CONFIG |
| `RESET` | CONFIG | Stage fresh defaults; EXIT discards them |
| `APPLY` | Except CALIBRATING | Save pending changes and reboot |
| `CALIBRATE` | RUNNING, NOT_CALIBRATED | Run isolated blocking calibration |
| `STOP` | Except CALIBRATING | Set voltage target to 0 |
| `mit_cmd: <pos> <vel> <torq> <p_gain> <v_gain>` | RUNNING | Apply the same MIT command as Cyphal |
| `servo_cmd: <type> <value>` | RUNNING | 0 velocity, 1 torque, 2 position, 3 voltage |
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
received during calibration is discarded, not executed afterwards. Wait for
completion before sending another command. Calibration cannot be cancelled.

Logging is disabled when leaving RUNNING and does not restart automatically.
Its format uses the position, velocity and torque fields of `voltbro.foc.State`,
in rad, rad/s and N m, without timestamp:

```text
state: 0.000000 0.000000 0.000000
```

Replies include `OK: mit_cmd`, `OK: servo_cmd`, `OK: STOP`, and
`OK: <register>:<value>`; errors begin with `ERROR:`.
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

<details>
  <summary>Note on backwards compatibility</summary>

  1. `State.1.0` preserves the first four fields of `state_simple.1.0` on the wire.
  Legacy clients can still read timestamp, angle, velocity and torque; the removed
  current, voltage, temperature and fault fields decode as zeros, not measurements.
  Read those measurements through the Cyphal registers instead.
  2. Old clients can still send the 28-byte legacy
`command.1.0`, motor will ignore its trailing `I_kp`/`I_ki`. Both formats leave the current
gains unchanged. This compatibility is for old clients
with new firmware; new clients with old firmware are not supported.

</details>

### **Published Messages**

| Port ID | Message Type                              | Interval | Description                                   |
| ------- | ----------------------------------------- | -------- | --------------------------------------------- |
| `3811`  | `voltbro.foc.State.1.0`               | 1 ms     | Timestamp, position, velocity, torque        |

---

### **Subscribed Messages**


| Port ID Formula  | Message Type                       | Description                                                                 |
| ---------------- | ---------------------------------- | --------------------------------------------------------------------------- |
| `2107 + node_id` | `voltbro.foc.MITCommand.1.0` | Torque, position, velocity, position gain and velocity gain |
| `3407 + node_id` | `voltbro.foc.Servo.1.0` | VELOCITY=0, TORQUE=1, POSITION=2, VOLTAGE=3; uint8 type, float32 value |


---

All registers are listed in the shared configuration/control and readonly tables
above. Cyphal persistent writes use the existing deferred EEPROM-save path.
An unconfigured device starts Cyphal in maintenance mode with a deterministic
setup node ID derived from the MCU UID.

### **Angle Frame Semantics**

> NOTE: angle offset only makes sense with ang_enc=1 (shaft output encoder). Rotor encoder (0) is not absolute in reference to shaft position

All joint-angle values exposed over Cyphal use the same corrected frame:

* `reported_angle = measured_shaft_angle * ang_dir + ang_off`
* `voltbro.foc.MITCommand.pos`, `voltbro.foc.Servo` position targets, `min_ang`, and `max_ang` are all interpreted in that corrected frame
* Positive `ang_off` increases the reported and commanded joint angle for the same physical shaft position
* Units are radians

This means limit enforcement and position control are applied after the offset is added, so the configured limits match the angles seen by higher-level kinematics.

### **Calibration Workflow**

1. Move the joint to the desired mechanical zero.
2. Read the current joint angle.
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

Initialize submodules before configuring. DSDL C headers and C++ traits are generated into the build directory using the CMake module and templates supplied by libcxxcanard. Neither the Arduino `src/` tree nor an `App/cyphal.h` shim is used.

`VBDrive_full.hex` combines VBBoot at `0x08000000` with VBDrive at `0x08003000`. Both Release and RelWithDebInfo fit the flash partitions; Debug is not supported on this layout.

Use Release for motor testing. The 2026-09-09 Servo smoke passed Serial/Cyphal
and bounded PI/PID movement, but sampled FOC peaks still exceeded the 25 us
budget (Release: 36.21 us; RelWithDebInfo: 39.56 us). Neither build has a verified
40 kHz worst-case deadline; see `tests/README.md` for the measurement scope.
Configure with `-DVBDRIVE_FOC_PROFILE=ON` to expose `last_cycle_cost` and
`max_cycle_cost` in Release without enabling MONITOR (sample period: 256 calls).

After a Release build, run `python3 tests/parameter_interfaces.py` for host regressions against the actual parameter implementation, Serial state controller and Cyphal callback (hardware/transport doubles).

The bootloader-compatible configuration prefix uses type `0x44AAABFF`. Only fresh
EEPROM provisioning is supported; existing configuration/calibration layouts are not migrated.
