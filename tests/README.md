# Firmware verification

Run host checks from the repository root after generating the Release DSDL headers.
Tests compile production implementations with hardware/transport doubles where
required. They do not establish physical motor behavior or MCU timing.

| Command | Coverage |
| --- | --- |
| `python3 tests/servo_input.py` | Standalone generators and abstract lifecycle: filter, ramp, trapezoidal/triangular profiles, reversal, overspeed, completion, invalid inputs and retargeting. |
| `python3 tests/servo_control.py` | Production FOC command handling and PID, DIRECT/MIT, optional generator, indices, limits, ang_dir, 1:8/1:5 scheduling, interrupt-safe publication and POLY time horizon. |
| `python3 tests/parameter_interfaces.py` | Production configuration, Serial controller and Cyphal register adapter: 44 registers, defaults, staging, persistence, shared command dispatch and FIFO framing. |
| `python3 tests/refactor_contracts.py` | Shared C/C++ configuration ABI, storage addresses, independent schema validation, staging, VBBoot baud/config loading, profiling counters and README defaults. |
| `python3 tests/mit_control.py` | Supported 20/28-byte MIT payloads and State serialization across CAN MTUs. |
| `python3 tests/kalman_filter.py` | Observer initialization, instance isolation, wrap/tracking and shaft angle from the encoder before filtering. |
| `python3 tests/cyphal_tx.py` | Bounded/full servicing, TX expiry, busy hardware FIFO and RX overwrite wrap. |
| `python3 tests/control_counters.py` | Production counters-only recording, accepted/rejected totals, one-second timestamps and ring wrap. |
| `python3 tests/millis_clock.py` | Millisecond polling/IRQ interleaving, single consumption of a timer tick, interrupt mask preservation and timestamp wrap. |
| `python3 tests/cyphal_arena.py` | Actual libcanard/O1Heap: release allocations, overwrite/reset shared arena, retain five subscription listeners and resume RX/TX. |

Build ordinary Release, CyphalProfile (`VBDRIVE_CYPHAL_PROFILE=ON`),
FOCProfile (`VBDRIVE_FOC_PROFILE=ON`) and Debug variants. Inspect linker Flash/RAM
usage. Profiling-disabled builds must contain no diagnostic counter state.
Configure both the application and VBBoot with the same `VB_CONFIG_ADDRESS`;
the parent build forwards it automatically. Configuration must not overlap
calibration or inductive sensor state.

## Hardware checks

Hardware scripts require explicit authorization for their effects and a verified
physical target, node ID and ST-Link serial. Preserve actual settings before
writes; complete testing with those settings restored and the driver disabled.
Allow firmware initialization to finish before starting a Serial test.

- `serial_hardware.py --port PORT --revision REV --output FILE` checks the live
  catalog, readonly/invalid requests, CONFIG/EXIT/RESET/SAVE/APPLY, runtime enable,
  logging and STOP. It reboots and enables/disables the driver.
- `serial_stress.py --port PORT --output FILE [--motion] --rate 100` exercises
  concurrent queries and state logging; `--motion` enables bounded targets.
- `cyphal_control_stream.py` sends timed MIT/Servo targets. Record both sender
  pacing and MCU arrival/handler intervals to distinguish delivery bursts from
  firmware execution time.
- Compare ordinary and profiled firmware under equal parameters, including all
  generator modes with zero/nonzero I and 1 kHz command/State traffic. Inspect
  rolling FOC frequency and reference/integral divider ratios separately.

Persistent configuration at address zero remains compatible across these builds.
Nonzero-address firmware must be provisioned at its selected address; changing the
build option does not relocate EEPROM data.

For a low-overhead measurement build, enable `VBDRIVE_COUNTERS_ONLY=ON` in Release
with FOC/Cyphal profiling disabled. It increments FOC, command and State counters
and records cumulative snapshots with MCU timestamps about once per second in
`counter_windows`. Counter differences divided by actual timestamp differences
give rates; select windows entirely inside the active input stream. No DWT reads,
handler histograms or RX-arrival profiling IRQ are enabled in this mode.
