## Why

VBDrive reports MCU and stator temperatures but does not use them to inhibit motor operation. A temperature excursion or invalid measurement must force the bridge OFF, retain communication/configuration access and permit a controlled neutral restart after recovery.

## What Changes

- Add persistent Serial/Cyphal `real32` registers `max_temp` and `restart_temp` in **kelvins**, defaulting to **363.15** and **333.15** for the stator. Existing `temp_mcu`/`temp_stator` readings remain in kelvins.
- Use explicit MCU junction-temperature constants of **233.15 K and 403.15 K**, from the operating range of the STM32G431VBx3 embedded in STSPIN32G4.
- Add a software thermal interlock: stator hysteresis, MCU range checking and invalid-sensor handling. Trip means actual OFF, with stale control cleared. Recovery can automatically re-enable a previously enabled drive at a neutral target; subsequent movement requires a fresh accepted command.
- Keep protection independent of `CommandState`. Introduce `enum class VBDriveStatus : uint8_t { NOMINAL = 0, COOLING_DOWN = 1 }`; CONFIG and protection can coexist.
- Publish the status through Heartbeat `vendor_specific_status_code`, and derive thermal health as CAUTION while inhibited, otherwise ADVISORY at/above stator restart temperature and NOMINAL below it.
- Guard startup, explicit enable, CONFIG restoration and calibration as well as normal motor operation. Invalid readings inhibit enable until valid recovery.
- **BREAKING:** Heartbeat vendor status becomes a defined 0/1 drive status instead of the current queue length.
- **BREAKING:** Two persistent fields require an application EEPROM schema revision. Change the application and calibration TYPE_ID values; incompatible application settings reset to defaults and old calibration is rejected. Configure the drive anew and recalibrate, without migration or automatic restoration. Preserve the common bootloader prefix and sensor region.

## Capabilities

### New Capabilities

- `temperature-protection`: Sensor validity, temperature bounds, hysteresis, independent protection status, automatic recovery and Heartbeat health/status semantics.

### Modified Capabilities

- `register-interface`: Fifty-register catalog, kelvin thermal thresholds, cross-field validation, live activation and application-schema version.
- `drive-lifecycle`: Thermal guards on enabling/restoration, protection-aware CONFIG behavior, calibration interruption and neutral recovery.
- `coordinates-and-units`: Kelvin units for both protection thresholds and temperature measurements.

## Impact

- VBDrive: `App/config/`, `App/state_manager/`, `App/app.cpp`, Serial/Cyphal interfaces and Heartbeat construction, README and host/hardware verification.
- `Drivers/libvoltbro` submodule: VBDrive-specific sensor validity/enable protection and cooperative cancellation of blocking calibration. The common FOC/PID mathematics and normal control frequencies are unchanged.
- VBBoot keeps the common configuration ABI and does not need an application-specific type update. Verify shared placement and prefix loading.
- No new Cyphal subscriptions, DSDL types, command-state values, runtime heap allocation or extra status registers. Existing `is_fault` hardware-placeholder semantics are outside this change; thermal status is reported through Heartbeat.
