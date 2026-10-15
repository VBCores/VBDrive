## Context

See [proposal.md](proposal.md) for the change scope. Temperature conversion currently lives in `VBInverter::update_temperature()` and is called from Cyphal State publication. Calibration blocks foreground servicing and temporarily disables Cyphal. The bridge has several enable paths: startup, is_on, CONFIG restoration and calibration. `VBDrive::stop()` already disables MOE and preserves the STSPIN logic supply.

Heartbeat currently carries a constant nominal health and TX queue size in its uint8 vendor field. The common configuration prefix is shared with VBBoot; the application configuration is a separate versioned payload.

## Goals / Non-Goals

**Goals:** One coherent thermal decision across every enable path; continued observation while OFF and in CONFIG; neutral automatic recovery; current status in Heartbeat; predictable foreground cost and testable boundary behavior.

**Non-Goals:** Current derating, a configurable MCU threshold, hardware DRV_FAULT integration, additional fault/status registers, DSDL changes, smoothing MCU recovery with an extra hysteresis, new command states or changes to FOC/PID mathematics. No physical destructive overtemperature test is required.

## Decisions

### 1. Operating limits and units

Use named `constexpr float` MCU limits with explicit `_K` units: 233.15f and 403.15f. These are limits on the MCU die measured by its internal sensor.

Sources:

- [STSPIN32G4 DS13630 Rev 3, §5.3](https://www.st.com/resource/en/datasheet/stspin32g4.pdf#page=26) identifies the embedded part as STM32G431VBx3 and refers to its datasheet.
- [STM32G431 DS12589 Rev 6, Table 17, p.71](https://www.st.com/resource/en/datasheet/stm32g431kb.pdf#page=71) gives the suffix-3 operating junction range −40…130 °C. Its Table 101 also distinguishes junction and ambient ratings.
- STSPIN32G4 Table 2 gives −40…125 °C **ambient** operation; its absolute-maximum junction value is 150 °C. Neither is the MCU's normal junction limit.
- Local HAL `TEMPSENSOR_CAL1_TEMP`/`TEMPSENSOR_CAL2_TEMP` are calibration points, not protection bounds. Temperature conversion remains based on the existing factory-calibrated calculation, with validity guards added.

Use kelvins for tracker inputs, comparisons, fixed limits and persistent thresholds, matching temperature telemetry. Convert the inverter's existing internal Celsius readings at the tracker boundary; keep the sensor conversion mathematics unchanged.

### 2. Minimal state ownership

Keep hardware-related thermal state with `VBDrive`, next to its existing enable/disable logic: active thresholds, a stator-hot latch and a coherent cached protection result. Use a small result/state representation rather than a separately allocated manager or duplicated per-transport logic. `DriveStateController` retains application-level recovery intent and CONFIG enable restoration, because only it knows whether the command state allows enabling.

Declare `enum class VBDriveStatus : uint8_t { NOMINAL = 0, COOLING_DOWN = 1 };`. Do not add COOLING_DOWN to CommandState. Heartbeat assigns `to_underlying(status)`; specifying uint8_t storage does not make a scoped enum implicitly convertible. No CommandState representation change is needed for this feature.

Expose thermal health through a transport-independent `enum class ThermalState : uint8_t` with NOMINAL, ADVISORY and CAUTION. Map it explicitly to UAVCAN Health in the application Cyphal layer; libvoltbro has no dependency on UAVCAN types.

The decision sequence is:

1. Validate both temperature conversions. An invalid snapshot inhibits operation; before the first valid sample, enable is inhibited.
2. For a valid stator reading, latch when T >= max; clear an existing latch only when T <= restart. Invalid samples do not erase an existing hot latch.
3. MCU inhibit is T < min or T > max, with inclusive in-range recovery.
4. Overall inhibit is invalid-input OR stator-hot OR MCU-out-of-range. It yields COOLING_DOWN/CAUTION; otherwise status is NOMINAL and health comes from the stator advisory band.

A new invalid reading alone does not manufacture a stator-hot latch. If validity returns at 348.15 K without a previous stator trip, recovery is allowed and health is ADVISORY. If 363.15 K had latched overheating, recovery still needs 333.15 K.

### 3. Health mapping and wire behavior

| Condition | VBDriveStatus / vendor byte | Heartbeat health |
| --- | --- | --- |
| Any thermal/validity interlock active | COOLING_DOWN / 1 | CAUTION / 2 |
| No interlock, stator >= restart_temp | NOMINAL / 0 | ADVISORY / 1 |
| No interlock, stator < restart_temp | NOMINAL / 0 | NOMINAL / 0 |

Use one evaluated snapshot for both fields. At default stator recovery exactly 333.15 K, the drive may re-enable while health is ADVISORY; below 333.15 K it is NOMINAL. MCU recovery with a cool stator can go directly from CAUTION to NOMINAL. CONFIG does not change this mapping.

Retain the existing one-second Heartbeat schedule and publish the current fields at its next slot. Do not send a burst of transition Heartbeats or change DSDL. Existing consumers interpreting the vendor byte as queue length must adopt the documented 0/1 status meaning. `is_fault` remains outside scope.

### 4. Scheduling and sensor validity

Move temperature conversion out of State publication into a foreground service, due once per new millisecond. State and register reporting read the latest result. Call the service independently of Cyphal availability and while OFF/CONFIG. A due service that discovers a trip disables the bridge immediately in that pass. Before any enable, refresh/evaluate current measurements instead of relying on an old healthy cache.

This retains the existing nominal conversion cadence and avoids adding conversion/logarithm work to the 40 kHz ISR. Evaluate into locals; use a short critical section for coherent status/control publication and the existing OFF operation. ADC conversion and trajectory/FOC math remain unchanged.

Validate stator raw values before logarithm/division: 0 or full-scale/out-of-range codes are invalid, as are non-finite results. Validate MCU VREF input and calibration denominator before conversion; non-finite results are invalid. Use the ADC resolution's actual endpoint definitions. This catches identified short/open endpoints without claiming to diagnose every disconnected sensor from a plausible in-range reading. The valid-input range must not confuse an ordinary cold stator with a sensor failure.

The one-millisecond schedule is a nominal foreground cadence, not proof of a hard interrupt response bound during arbitrary blocking operations. Measure service spacing and trip latency under the required steady-state workload. Service operations that energize the motor must explicitly retain checks; service operations with PWM already OFF can delay observation but cannot bypass the fresh enable gate.

### 5. Enable gate, intent and CONFIG

Make the VBDrive enable path reject active protection before MOE can be enabled. App-level callers consult the same result, including startup and CONFIG restoration. Audit PWM-channel initialization as well: HAL PWM start calls can enable MOE implicitly, so setup must leave the bridge inhibited until the first successful thermal gate, rather than guarding only the final start() call. On a trip, call actual OFF, retain only the preceding enabled intent, and clear old motion/reference/integral/index state. A normal motion command rejected during protection never becomes a queued recovery target.

`DriveStateController` owns pending neutral restoration. Explicit is_on=0 clears both thermal restoration intent and the existing CONFIG restore flag, even if the bridge is already OFF. A rejected explicit ON does not create new intent. Startup that would normally enable can retain startup intent after a hot-boot rejection, but calibration validity and command-state checks still govern later enable.

CONFIG temporarily prevents automatic enable without erasing a pre-existing restore intent. SAVE/EXIT restores only in RUNNING and with valid recovered temperatures. SAVE entered from NOT_CALIBRATED preserves that state so configuration cannot bypass calibration eligibility. If still interlocked, successful configuration completion leaves PWM OFF and defers that restoration; it is not a gate-driver failure. Once restored successfully, clear pending intent. The normal start/bootstrap path provides neutral output and requires a fresh accepted motion command.

### 6. Blocking calibration

A foreground-only tracker cannot protect the current blocking calibration. Add a cooperative continuation/cancellation hook to calibration, preserving the existing progress callback. Check before energizing a calibration step and during waits, splitting long waits into at most 10 ms intervals while energized. Abort promptly when the hook reports thermal inhibition; the low-level OFF gate prevents subsequent PWM writes from re-energizing the bridge.

Propagate an explicit unsuccessful outcome through `do_calibrate()`. On thermal abort: disable PWM, clear auto-resume intent for that interrupted calibration, restore communications/shared-memory ownership, report a command-prefixed error, and do not write or mark partial calibration successful. Enter NOT_CALIBRATED because in-memory calibration may have been modified. Preserve the previous EEPROM record; an explicit successful calibration or reboot that reloads valid stored calibration of the new type can restore normal eligibility. Do not automatically restart a calibration sequence after cooling.

### 7. Configuration and compatibility

Append max_temp/restart_temp real32 definitions and persistent application fields, default 363.15/333.15 K. The complete resolved pair must be finite and satisfy 0 < restart < max. NaN restores the individual default. For Serial, allow temporary ordering inconsistencies while staging individually valid values, then reject an invalid pair at SAVE/APPLY before persistence, runtime effects or reboot. Cyphal writes validate the resulting pair atomically against committed values, including before the motor exists; clients adjust fields in a valid order. Parameter writes distinguish staged edits from committed writes, and apply runtime changes only for committed writes with an available motor.

Apply successful Cyphal writes immediately and apply Serial thresholds on SAVE as well as restart. Reevaluate protection with the new limits without resetting the stator latch. EXIT preserves the previous committed thermal configuration. A change in thresholds can legitimately cause immediate trip or recovery.

Bump the application schema from 0x44AAAC04 to 0x44AAAC05. Also bump CalibrationData::TYPE_ID from 0x89ABCDEF to 0x89ABCDF0, requiring fresh calibration through the existing type check. Calibration is independent: changing the application TYPE_ID alone does not invalidate it. The calibration type change also affects consumers rebuilt with this shared library. The planned packed app payload grows from 122 to 130 bytes; common-plus-app grows from 150 to 158 bytes. Retain common type 0x01234567, configurable storage base, calibration at 0x0100 and sensor state at 0x2200, and verify placement bounds. VBBoot's common-prefix parsing does not depend on this app schema. Do not migrate or automatically restore old settings or calibration. Load application defaults, configure anew and recalibrate. Old calibration bytes need not be erased.

During spec synchronization, add the two rows to the main register catalog and retain all existing entries; the new-register requirement supplies their attributes. Document both threshold and measurement units as K.

## Risks / Trade-offs

- Sensor noise near an MCU boundary can cause repeated OFF/recovery because MCU hysteresis was not requested → test exact boundary behavior and report physical observations without adding a hidden threshold.
- Automatic recovery can energize PWM → preserve explicit OFF intent, apply command-state/calibration eligibility and reset to neutral; do not replay motion commands.
- Blocking calibration bypasses normal foreground service → cooperative cancellation and no partial calibration commit are required acceptance cases.
- A valid-looking NTC value can conceal a wiring/nominal-value fault → detect invalid conversions/endpoints and document the measurement assumption; broad sensor diagnostics are out of scope.
- An incompatible EEPROM app schema resets application settings → configure anew and recalibrate; verify the common prefix and sensor data remain intact.

## Deployment

1. Build and validate the new configuration and calibration type IDs, EEPROM layout and 50-register catalog. No migration or old-settings restoration machinery is required.
2. Deploy to the explicitly selected drive; verify application defaults and rejection of old calibration. Configure the drive anew and recalibrate.
3. Verify 363.15/333.15 K defaults, new configuration/calibration persistence and preservation of the common prefix and sensor region.
4. Test protection with controlled thresholds or isolated sensor injection. Restore the newly configured drive's settings after temporary tests and leave it OFF.
