# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## Configuration Parameters (Serial and Cyphal)

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

### Servo Parameters

Servo parameters are shared Serial/Cyphal read/write persistent registers:

| Name | Type | Default |
| --- | --- | --- |
| `servo_pos_p_gain` | real32 | 0 |
| `servo_pos_i_gain` | real32 | 0 |
| `servo_vel_p_gain` | real32 | 0 |
| `servo_vel_i_gain` | real32 | 0 |
| `servo_tr_form` | natural32 | 1 (`LINE_TRAJ`; 2 = `POLYNOM_TRAJ`) |
| `servo_tr_vel` | real32 | 0 |

Gains and transient velocity must be finite and non-negative. This change implements
communication and persistence only: these values do not yet alter Servo control or
generate trajectories. Existing Servo setpoint handling is unchanged. Its wire format
matches legacy `specific_control` for modes 0–3; other modes are rejected.
Serial writes require CONFIG and SAVE/APPLY; EXIT discards them. Cyphal writes are
saved by the existing deferred-save path. Names are string views (up to 16 characters),
not heap-allocated strings. The composite `servo_params` register is not used.

All settings, including Servo, are stored in one 98-byte config at EEPROM offset 0.
Calibration starts at 99, followed by encoder state. There is no old-layout migration.
Provision devices with erased external EEPROM, then configure and calibrate afresh;
flashing MCU firmware alone does not erase external EEPROM.


---

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

The board uses a UART-based serial interface for configuration, calibration, test control, and debug logging.

### **Connection Details**

* **Baud Rate**: 19200
* **Format**: ASCII commands; trailing `\r`, `\n`, spaces and tabs are stripped automatically

---

### **Command Syntax**

* **Read parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` -> `node_id:11`. Queries work in every mode, without `CONFIG`.

* **Write parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` -> `OK: kp:0.350000`

---

### **Mode and Command Overview**

| Command | Available State | Description |
| ------- | --------------- | ----------- |
| `CONFIG` | Any non-TEST state | Enter configuration mode and stop motor |
| `EXIT` | CONFIG | Discard staged changes without writing EEPROM; restore the previous mode |
| `SAVE` | CONFIG | Persist updated config to EEPROM (if changed), exit config mode, start motor |
| `RESET` | CONFIG | Load default config values in RAM (does NOT affect current session - requires `SAVE` or `APPLY` to persist) |
| `APPLY` | Any non-TEST state | Persist updated config (if changed) and reboot |
| `BOOT` | Any state | Write bootloader request magic and reboot into VBBoot |
| `CALIBRATE` | RUNNING, NOT_CALIBRATED | Run calibration action |
| `TEST` | RUNNING | Enter test mode |
| `STOP` | TEST | Exit test mode, clear FOC target, stop test logging |

---

### **TEST Mode Commands**

| Command | Description |
| ------- | ----------- |
| `do_vel:<value>` | Set velocity target for FOC test controller |
| `do_ang:<value>` | Set angle target for FOC test controller |
| `do_free` | Zero target (no effort mode) |
| `min_ang:<value or ?>` | Read/write lower position limit during test |
| `max_ang:<value or ?>` | Read/write upper position limit during test |
| `ang_off:<value or ?>` | Read/write angle offset during test |
| `log_on` | Start UART test logging |
| `log_off` | Stop UART test logging |
| `STOP` | Exit test mode |

When logging is enabled in TEST mode, UART periodically prints:

```text
rotor: <u16> shaft :<u16> angle: <float> velocity: <float>
```

---

### **Response Format**

* **Success**: `OK: <param>:<value>` (set operations)
* **Error**: `ERROR: Unknown command`, `ERROR: Unknown parameter`, `ERROR: Invalid value`
* **Config persistence**: UART configuration settings are written to EEPROM on `SAVE`/`APPLY` (not on every `SET`)
* **Bootloader entry**: `BOOT` writes only the bootloader request magic. VBBoot reads `node_id`, `nominal_baud`, and `data_baud` from the EEPROM config prefix.

---

### **Example Sessions**

```bash
# Configuration flow
> CONFIG
CONFIG MODE ENABLED
> node_id:11
OK: node_id:11
> gear:36
OK: gear:36
> SAVE
Saved config
NOTE: config changes not applied! To apply, run APPLY or reset controller
> APPLY
```

```bash
# Test flow
> TEST
Entering TEST mode
> do_vel:5
Set velocity: <5.000000>
> log_on
# periodic sensor logs...
> STOP
Stopping TEST mode
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

### **Registers**

All configuration parameters are shared between Cyphal and Serial interfaces, except `bootloader` and `cmd_errors`, which are Cyphal-only. All config params listed at the top of README are mutable and persistent. Integer parameters use `natural32`, except `ang_dir` (`integer32`); floating-point parameters use `real32`.

The remaining runtime registers are non-persistent and Cyphal-exclusive:

| Name | Cyphal type | Access | Meaning |
| --- | --- | --- | --- |
| `is_on` | bit | read/write | Driver enable; Serial writes 0/1 in RUNNING |
| `bootloader` | bit | read/write | Writing `true` reboots drive into VBBoot bootloader |
| `cmd_errors` | natural32 | read-only | Cyphal-only; number of rejected Cyphal movement commands |
| `vbdrive_model` | string | read-only | CMake constant `M4310` |
| `firmware_rev` | string | read-only | 16 hexadecimal digits of the VBDrive HEAD commit |
| `bus_voltage` | real32 | read-only | Bus voltage, V |
| `bus_current` | real32 | read-only | Existing working-current measurement, A; not a separate DC-link current sensor |
| `temp_mcu` | real32 | read-only | MCU temperature, K |
| `temp_stator` | real32 | read-only | Stator temperature, K |
| `is_fault` | bit | read-only | Currently false; DRV_FAULT integration is deferred |
| `encoder_shaft` | natural32 | read-only | Raw external encoder counts |
| `encoder_rotor` | natural32 | read-only | Raw internal encoder counts |

Readonly writes do not change values. Unavailable motor measurements return an empty Cyphal value / a Serial error before motor initialization. `firmware_rev` also supplies GetInfo's numeric `software_vcs_revision_id`; it identifies the commit, not uncommitted changes.

Serial config writes are staged until `SAVE`/`APPLY`. Repeated `CONFIG` does not replace the rollback snapshot; `EXIT` also rolls back `RESET`. TEST retains live `min_ang`, `max_ang`, `ang_off` updates. Reads work in every mode.

Persistent register writes are queued and saved to EEPROM from the main loop. The config starts at EEPROM offset `0`, so VBBoot can read the shared C-compatible prefix containing `node_id`, `nominal_baud`, and `data_baud`. If the app does not have a complete EEPROM config yet, it starts Cyphal in maintenance mode with a deterministic setup node ID derived from the MCU UID.

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

After a Release build, run `python3 tests/parameter_interfaces.py` for host regressions against the actual parameter implementation, Serial state controller and Cyphal callback (hardware/transport doubles).

The bootloader-compatible configuration prefix uses type `0x44AAABFF`. Only fresh
EEPROM provisioning is supported; existing configuration/calibration layouts are not migrated.
