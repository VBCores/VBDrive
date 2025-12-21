# STM32 BLDC Motor Controller

> This is a repo for VBDrive firmware
>
> [Buy here]()

## UART Configuration Interface

The board uses a UART-based serial interface to configure parameters and control system behavior.

### **Connection Details**

* **Baud Rate**: 19200
* **Format**: Commands are ASCII strings, terminated with `\r`, `\n`, or spaces (automatically stripped)

---

### **Command Syntax**

* **Enter Config Mode**:
  `START` — Enables configuration mode (motor is stopped while in config mode)

* **Get Parameter**:
  `<parameter_name>:?`
  Example: `node_id:?` → `node_id:11`

* **Set Parameter**:
  `<parameter_name>:<value>`
  Example: `kp:0.35` → `OK: kp :0.350000`

* **System Commands**:

  * `APPLY` — Reboots the controller to apply changes
  * `RESET` — Resets all configuration to defaults (requires `APPLY` after)
  * `STOP` — Exits config mode without applying changes

---

### **Configuration Workflow**

1. **Enter Config Mode**
   Send `START` to display current settings and unlock configuration.

2. **Modify Parameters**
   Use `<param>:<value>` syntax to adjust settings.

3. **Save & Apply Changes**

   * Send `APPLY` to reboot and apply changes
   * Or send `STOP` to exit config mode (changes will apply after next reboot)

---

### **Parameters**

| Parameter      | Description                                         | Type    | Example Values |
| -------------- | ----------------------------------------------------| ------- | -------------- |
| `gear_ratio`   | Gear ratio of the drive                             | Integer | `1`, `5`, `15` |
| `max_current`  | Maximum motor current (A)                           | Float   | `5.0`, `10.5`  |
| `max_speed`    | Maximum motor speed (rad)                           | Float   | `1000.0`       |
| `max_torque`   | Maximum torque output (Nm)                          | Float   | `1.2`          |
| `angle_offset` | Motor angle offset (rad)                            | Float   | `0.0`, `15.5`  |
| `min_angle`    | Minimum allowed angle (rad)                         | Float   | `-30.0`        |
| `max_angle`    | Maximum allowed angle (rad)                         | Float   | `30.0`         |
| `torque_const` | Torque constant (Nm/A)                              | Float   | `0.12`         |
| `kp`           | Proportional gain (not for FOC, *specific control*) | Float   | `0.25`         |
| `ki`           | Integral gain (not for FOC, *specific control*).    | Float   | `0.01`         |
| `kd`           | Derivative gain (not for FOC, *specific control*)   | Float   | `0.005`        |
| `filter_a`     | Main filter parameter A                             | Float   | `0.5`          |
| `filter_g1`    | Filter gain 1                                       | Float   | `0.1`          |
| `filter_g2`    | Filter gain 2                                       | Float   | `0.1`          |
| `filter_g3`    | Filter gain 3                                       | Float   | `0.1`          |
| `node_id`      | Cyphal/CAN node ID                                  | Integer | `1`, `42`      |
| `data_baud`    | FDCAN data baud rate (enum, see below)              | Enum    | `KHz1000`      |
| `nominal_baud` | FDCAN nominal baud rate (enum, see below)           | Enum    | `KHz500`       |

---

### FDCAN Baud Rate Configuration

| parameter           | Value Name | Speed    | Numeric Value |
|---------------------|------------|----------|---------------|
| `nominal_baud`      | `KHz62`    | 62.5 kHz | `0`           |
|                     | `KHz125`   | 125 kHz  | `1`           |
|                     | `KHz250`   | 250 kHz  | `2`           |
|                     | `KHz500`   | 500 kHz  | `3`           |
|                     | `KHz1000`  | 1 MHz    | `4`           |
|---------------------|------------|----------|---------------|
| `data_baud`         | `KHz1000`  | 1 MHz    | `0`           |
|                     | `KHz2000`  | 2 MHz    | `1`           |
|                     | `KHz4000`  | 4 MHz    | `2`           |
|                     | `KHz8000`  | 8 MHz    | `3`           |

---

### **System Commands**

| Command | Description                                             |
| ------- | ------------------------------------------------------- |
| `START` | Enter config mode, motor stopped                        |
| `APPLY` | Reboot and apply changes                                |
| `RESET` | Reset config to defaults (needs `APPLY` to take effect) |
| `STOP`  | Exit config mode without applying changes               |

---

### **Response Format**

* **Success**: `OK: <param> :<value>` (for set operations).
* **Error**: `ERROR: <reason>` (e.g., `ERROR: Invalid value`).
* **Auto-Save**: Changes are saved to EEPROM automatically after valid `SET` commands

---

### **Example Session**

```bash
# Enter config mode
> START
CONFIG MODE ENABLED
gear_ratio:5
max_current:10.500000
...

# Set max speed
> max_speed:1200
OK: max_speed :1200.000000

# Apply changes
> APPLY
```

## FDCAN Cyphal Runtime Interface

Here’s the **Cyphal Runtime Interface** section for the BLDC Motor Controller README:

---

## Cyphal Runtime Interface

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

| Register Name | Type   | Description                                         |
| ------------- | ------ | --------------------------------------------------- |
| `motor.is_on` | `bool` | Turns underlying motor driver on/off                |

---

### **Standard Cyphal Services**

The controller also supports standard Cyphal services:

* **uavcan.node.GetInfo** — Reports node information (name: `"org.voltbro.vbdrive"`)
* **uavcan.register.Access** — For reading/writing runtime parameters
* **uavcan.register.List** — List registers
* **uavcan.node.Heartbeat** — Node status monitoring

### **Standard Cyphal Messages**

The controller also publishes standard Cyphal messages:

* **uavcan.node.Heartbeat** - default heartbeat message

---
