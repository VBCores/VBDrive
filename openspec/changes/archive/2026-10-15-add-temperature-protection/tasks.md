## 1. Configuration and public contract

- [x] 1.1 Add `max_temp`/`restart_temp` to the shared register catalog and application config as persistent real32 kelvin values with defaults 363.15/333.15; preserve all existing register attributes.
- [x] 1.2 Validate finite resolved values and the final threshold ordering; support individual NaN-to-default reset, valid staged Serial pairs and atomic Cyphal candidate validation. Reject invalid SAVE/APPLY before mutation/persistence/reboot.
- [x] 1.3 Apply successful Cyphal threshold writes and Serial SAVE live, retaining hysteresis state and reevaluating it under the new limits.
- [x] 1.4 Bump the application type to 0x44AAAC05 and CalibrationData::TYPE_ID to 0x89ABCDF0; require fresh configuration and calibration without migration. Verify the 130-byte app/158-byte total layout, unchanged common prefix, storage-address bounds and non-overlap with calibration/sensor data.

## 2. Temperature tracking and bridge gating

- [x] 2.1 Add the documented MCU kelvin limits (233.15/403.15 K) and uint8-backed `VBDriveStatus` with explicit NOMINAL=0/COOLING_DOWN=1. Keep CommandState unchanged.
- [x] 2.2 Add sensor-conversion validity checks for uninitialized/unavailable data, stator raw endpoints, MCU reference/calibration inputs and non-finite results; keep valid temperature conversion and kelvin telemetry compatible.
- [x] 2.3 Implement stator hysteresis, MCU range and invalid-sensor inhibition using the decision sequence in design.md. Publish a coherent status/health result.
- [x] 2.4 Move the existing temperature conversion cadence to an independent nominal 1 ms foreground service that also runs while OFF/CONFIG and without Cyphal. Keep conversion outside the 40 kHz control path.
- [x] 2.5 Gate every bridge-enable path on initialized current measurements, including implicit MOE changes during initialization, normal start/bootstrap, explicit enable, CONFIG restoration and calibration entry.
- [x] 2.6 Implement forced OFF and neutral automatic recovery with retained enabled intent. Preserve CONFIG coexistence, defer restoration until RUNNING and clear pending restoration on explicit OFF. Reject movement commands during inhibition without retaining targets.

## 3. Calibration and Heartbeat integration

- [x] 3.1 Add cooperative thermal cancellation to blocking calibration and bound checks/wait segments to 10 ms while energized; retain existing progress reporting.
- [x] 3.2 Propagate thermal abort without marking/saving partial calibration, restore communications/shared-memory ownership, cancel calibration auto-resume and report the specified error with NOT_CALIBRATED state.
- [x] 3.3 Populate scheduled Heartbeat health and vendor_specific_status_code from one thermal snapshot using the specified mapping and explicit enum conversion. Preserve the existing cadence, DSDL, subscriptions and Cyphal mode handling.

## 4. Host verification

- [x] 4.1 Test stator exact boundaries and the full 332.15→333.15→363.15→353.15→333.15→332.15 K cycle, verifying latch, actual OFF, neutral re-enable and matching Heartbeat health/vendor values.
- [x] 4.2 Test MCU low/high excursions, inclusive recovery, simultaneous causes, invalid-to-valid recovery with and without a previous stator latch, raw open/short endpoints and an initially invalid snapshot.
- [x] 4.3 Test startup protection, already-OFF behavior, explicit OFF cancelling restoration, rejected ON during inhibition, CONFIG entry/EXIT/SAVE during cooling, and cooling completion while CONFIG remains active.
- [x] 4.4 Test no replay of pre-trip motion/profile/index/integral state, movement rejection during inhibition, and availability of queries/configuration/logging under the stated command-state rules.
- [x] 4.5 Test valid/invalid threshold pairs through both transports, serial staging/commit failure, NaN defaults, live threshold trip/recovery, persistence across reboot, 50-register enumeration and common-prefix/sensor preservation, rejection of old application/calibration types and persistence of fresh calibration at supported storage addresses.
- [x] 4.6 Test thermal cancellation during calibration steps/waits, no partial EEPROM commit or FINISH report, communication restoration and continued inhibition until recovery.
- [x] 4.7 Run affected existing regressions: temperature_conversion, parameter_interfaces, refactor_contracts, servo_control, servo_input, rated_limits, mit_control, bus_current, pwm_foc_sync, millis_clock and cyphal_arena; add timing/Heartbeat checks that exercise production paths.

## 5. Build and physical acceptance

Physical target: VBDrive 03 (ST-Link 066DFF544983555067233335), selected by the user on 2026-10-15.

- [x] 5.1 Build ordinary Release, Debug and relevant profiling/counters variants; inspect Flash/RAM and confirm no new runtime heap allocation or Cyphal subscription.
- [x] 5.2 On the explicitly selected drive, deploy, verify application defaults and rejection of old calibration, configure anew and recalibrate. Verify live threshold access and persistence through Serial/Cyphal.
- [x] 5.3 Exercise trip/cooling/recovery using controlled thresholds or isolated sensor injection. Observe is_on, PWM MOE, neutral recovery and actual Heartbeat values. Do not heat the MCU beyond its operating range.
- [x] 5.4 Measure temperature-service spacing/trip latency, FOC rates and command/State traffic under the existing 1 kHz workload with nonzero I and changing generator goals. Compare ordinary and instrumented builds without reducing control rates.
- [x] 5.5 Verify calibration protection with a controlled test and retained valid new-type EEPROM calibration, then restore the newly configured drive’s settings after temporary tests and leave is_on=0/MOE=0. Keep hardware evidence distinct from host results.

## 6. Documentation and completion

- [x] 6.1 Update README with kelvin thresholds/defaults and readings, protection/recovery semantics, independent VBDriveStatus and Heartbeat mapping, MCU source citations and fresh-configuration/recalibration instructions without migration.
- [x] 6.2 Review final behavior against every delta scenario, record actual validation evidence, run strict OpenSpec validation and prepare the completed change for archive/synchronization. Include the two new rows when synchronizing the main register catalog.

## Verification evidence

- Fifteen host checks passed; ordinary Release, Debug, combined FOC/Cyphal profiling and counters builds succeeded.
- Physical acceptance used VBDrive 03 selected by the user. Full calibration, threshold/Heartbeat/MOE checks, invalid MCU/NTC injection and thermal calibration abort/reload passed.
- FOC remained approximately 40 kHz. Changing POLY goals at nominal 1 kHz exposed a foreground throughput limit; detailed profiling adds further overhead. Sender/arrival gaps and acceptance totals are recorded separately, and universal lossless 1 kHz/jitter qualification is not claimed.
- A separate existing newlib strtof heap-allocation failure on a long decimal Serial value was diagnosed and recorded; the numeric parser was not changed by this feature.
- Final hardware state: ordinary Release, gear=10, kt=0.75, restored default settings and thermal thresholds, valid new-type calibration, is_on=0 and MOE=0.
- Full report: [verification.md](../../../../var/temperature-protection-20261015/verification.md); raw host/hardware results are beside it.
- Main specs synchronized on 2026-10-15: all four delta capabilities verified, 50 register rows present, and all eight main specs passed strict validation. All 27 tasks are complete.
