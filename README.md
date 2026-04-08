# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## UART Configuration / Test Interface

The board uses a UART-based serial interface for configuration, calibration, test control, and debug logging.

### **Connection Details**

* **Baud Rate**: 19200
* **Format**: ASCII commands; trailing `\r`, `\n`, spaces and tabs are stripped automatically

---

### **Command Syntax**

* **Read parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` -> `node_id:11`

* **Write parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` -> `OK: kp:0.350000`

---

### **Mode and Command Overview**

| Command | Available State | Description |
| ------- | --------------- | ----------- |
| `CONFIG` | Any non-TEST state | Enter configuration mode and stop motor |
| `SAVE` | CONFIG | Persist updated config to EEPROM (if changed), exit config mode, start motor |
| `RESET` | CONFIG | Load default config values in RAM (does NOT affect current session - requires `SAVE` or `APPLY` to persist) |
| `APPLY` | Any non-TEST state | Persist updated config (if changed) and reboot |
| `CALIBRATE` | RUNNING, NOT_CALIBRATED | Run calibration action |
| `TEST` | RUNNING | Enter test mode |
| `STOP` | TEST | Exit test mode, clear FOC target, stop test logging |

---

### **Configuration Parameters**

| Parameter  | Description                                               | Type    | Example Values |
| ---------- | --------------------------------------------------------- | ------- | -------------- |
| `gear`     | Gear ratio of the drive                                   | Integer | `1`, `5`, `15` |
| `max_i`    | Maximum motor current (A)                                 | Float   | `5.0`, `10.5`  |
| `max_spd`  | Maximum motor speed (rad)                                 | Float   | `1000.0`       |
| `max_tq`   | Maximum torque output (Nm)                                | Float   | `1.2`          |
| `ang_off`  | Joint angle offset (rad)                                  | Float   | `0.0`, `15.5`  |
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
| `d_baud`   | FDCAN data baud rate enum (see below)                     | Enum    | `0`, `1`, `2`  |
| `n_baud`   | FDCAN nominal baud rate enum (see below)                  | Enum    | `3`, `4`       |

---

### **FDCAN Baud Rate Configuration**

| parameter           | Value Name | Speed    | Numeric Value |
|---------------------|------------|----------|---------------|
| `n_baud`            | `KHz62`    | 62.5 kHz | `0`           |
|                     | `KHz125`   | 125 kHz  | `1`           |
|                     | `KHz250`   | 250 kHz  | `2`           |
|                     | `KHz500`   | 500 kHz  | `3`           |
|                     | `KHz1000`  | 1 MHz    | `4`           |
|---------------------|------------|----------|---------------|
| `d_baud`            | `KHz1000`  | 1 MHz    | `0`           |
|                     | `KHz2000`  | 2 MHz    | `1`           |
|                     | `KHz4000`  | 4 MHz    | `2`           |
|                     | `KHz8000`  | 8 MHz    | `3`           |

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

* `rotor sensor: <u16>`
* `shaft sensor: <u16>`
* `shaft angle : <float>`

---

### **Response Format**

* **Success**: `OK: <param>:<value>` (set operations)
* **Error**: `ERROR: Unknown command`, `ERROR: Unknown parameter`, `ERROR: Invalid value`
* **Config persistence**: UART configuration settings are written to EEPROM on `SAVE`/`APPLY` (not on every `SET`)

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

### **Published Messages**

| Port ID | Message Type                              | Interval | Description                                   |
| ------- | ----------------------------------------- | -------- | --------------------------------------------- |
| `3811`  | `voltbro.foc.state_simple.1.0`            | 1 ms     | Current state (angle, speed, torque, etc.)    |

---

### **Subscribed Messages**

| Port ID Formula  | Message Type                       | Description                                                                 |
| ---------------- | ---------------------------------- | --------------------------------------------------------------------------- |
| `2107 + node_id` | `voltbro.foc.command.1.0`          | Direct FOC target command: torque, angle, velocity, and PID gains           |
| `3407 + node_id` | `voltbro.foc.specific_control.1.0` | High-level, single parameter control setpoint                               |

---

### **Registers**

| Register Name     | Type        | Persistence | Description                          |
| -------------     | ----        | ----------- | -----------                          |
| `state.is_on`     | `bool`      | Runtime     | Turns underlying motor driver on/off |
| `state.errors`    | `natural32` | Runtime     | Invalid Cyphal command counter       |
| `limit.current`   | `real32`    | EEPROM      | Current limit in amperes             |
| `limit.torque`    | `real32`    | EEPROM      | Torque limit in N m                  |
| `limit.speed`     | `real32`    | EEPROM      | Speed limit in rad/s                 |
| `limit.min_angle` | `real32`    | EEPROM      | Lower joint angle limit in rad       |
| `limit.max_angle` | `real32`    | EEPROM      | Upper joint angle limit in rad       |
| `angle.offset`    | `real32`    | EEPROM      | Joint angle offset in rad            |

### **Angle Frame Semantics**

> NOTE: angle offset only makes sense with ang_enc=1 (shaft output encoder). Rotor encoder (0) is not absolute in reference to shaft position

All joint-angle values exposed over Cyphal use the same corrected frame:

* `reported_angle = measured_shaft_angle + angle.offset`
* `voltbro.foc.command.angle`, `voltbro.foc.specific_control` position targets, `limit.min_angle`, and `limit.max_angle` are all interpreted in that corrected frame
* Positive `angle.offset` increases the reported and commanded joint angle for the same physical shaft position
* Units are radians

This means limit enforcement and position control are applied after the offset is added, so the configured limits match the angles seen by higher-level kinematics.

### **Calibration Workflow**

1. Move the joint to the desired mechanical zero.
2. Read the current joint angle.
3. Compute the required offset so the reported angle becomes zero:
   `angle.offset = -measured_shaft_angle`
4. Write `angle.offset` via `uavcan.register.Access`.
5. Read back `angle.offset`, `limit.min_angle`, and `limit.max_angle` to confirm the corrected frame.
6. Set `limit.min_angle` and `limit.max_angle` in the same corrected frame.

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
