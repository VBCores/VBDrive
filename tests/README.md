# Parameter integration verification

## MIT compatibility

After building RelWithDebInfo, run `python3 tests/mit_control.py`. This host test
uses the generated MITCommand/State/legacy serializers, the actual FOC handler and
libcanard TX/RX with MTU 8, 12, 16, 20, 24, 32, 48 and 64. It verifies the shared
20-byte prefix, CAN FD padding and RX extent truncation, all five target fields,
ignored legacy current-gain fields, unchanged current gains for both formats and
invalid-target error counting. It also publishes State into a legacy
state_simple subscriber, checking its four retained fields and zero-filled tail
for every MTU. No hardware or motion is involved.

## Parameters

Run `python3 tests/parameter_interfaces.py` after generating the Release DSDL headers.
The test compiles the actual parameter implementation, Serial controller and Cyphal
register callback with host hardware/transport doubles. It checks all 39 reads and
types (37 shared parameters plus Cyphal-only `bootloader` and `cmd_errors`),
rejection of both Cyphal-only names in every Serial mode, every persistent
parameter's Serial write/EXIT rollback and Cyphal write,
readonly write rejection, CONFIG snapshots, RESET rollback, SAVE after an invalid
write, TEST commands/live angle limits, numeric validation, delayed EEPROM writes,
configuration before motor creation and both boot commands. The EEPROM config size
and placements are checked (98 bytes; config 0, calibration 99).
Unified config writes/readback and preservation of bytes outside the config
are tested with a byte-addressed EEPROM double. Servo registers
are included in the write/rollback tests, with negative/non-finite gain rejection
and transient-form validation. The communication test also checks all four Servo
setpoint modes, invalid modes and wire equality with legacy specific_control.
It does not simulate the FOC loop, physical encoders, UART DMA or CAN transport.

## Main synchronization check, 2026-09-08

- VBDrive base: `85039e4`; previous local commit `921fa6c` rebased to `3226524`.
- libcxxcanard: `d6c2ae4`, matching its origin/master.
- libvoltbro: `64d23c4`, matching its origin/master.
- VBBoot: `2adc0bf`, matching its origin/main. The STM32G431 defaults, application
  start (`0x08003000`), 8-byte EEPROM prefix, config type (`0x44AAABFF`) and boot
  request magic remain compatible with current VBDrive main.
- Release and a clean RelWithDebInfo application plus bootloader and merged HEX build successfully with
  CubeCLT 1.16.0 / GCC 12.3.1. Generated native C++ traits include
  `cyphal/types.hpp`; Arduino packed headers and the old App shim are not used.
- Application Flash/RAM: Release 104468/28544 bytes; RelWithDebInfo 102488/28696
  bytes. VBBoot is 9352 bytes in both. The application has 116 KiB Flash / 32 KiB RAM.
- Two upstream build integration defects were corrected locally: the missing
  DSDL CMake module path and the missing Ninja HEX byproduct declaration.
- Host parameter regressions pass. The subsequent live check below includes flashing
  after backing up Flash and EEPROM and migrating the old EEPROM layout.

## Live hardware smoke, 2026-09-08

- Flashed and verified RelWithDebInfo merged HEX on ST-Link `0672FF544983555067215514`.
  Running revision: `32265249a8a7a070` (base commit; functional changes uncommitted).
- All 33 Serial reads, readonly rejection, CONFIG/EXIT and RESET rollback, SAVE,
  APPLY/reboot, TEST/logging passed. Final manual EXIT check restored kp=4.
- Cyphal GetInfo/list/access and bounded register writes passed; telemetry ~1000 Hz,
  no invalid/backward samples in the recorded readonly run.
- Bounded Serial and universal FOC velocity/position/torque smoke passed; specific
  torque/voltage responded. Specific velocity/position did not move with their
  zero controller gains; this is not claimed as successful motion validation.
- Serial BOOT entered VBBoot; a framed unsupported command returned ERR (0xE0)
  without writing Flash. ST-Link reset returned to the application. The Cyphal
  bootloader=true hardware path was not separately exercised.
- Final readback: gear=36, kp=4, ki=1600, ang_dir=1, cmd_errors=0, is_on=0.
  Motor left disabled. DRV_FAULT remains deferred; no recalibration performed.
- Calibration and sensor state were copied byte-for-byte during EEPROM migration,
  with full migration readback verified. Persistent write tests restored effective
  values, but materialized some NaN-backed defaults; final EEPROM is not claimed
  byte-identical to the migration snapshot.
- Backups, Serial/Cyphal reports and movement results are under
  `build/hardware-validation-20260908/`. These local build artifacts are not tracked.

Known upstream verification failures, not hidden by the port:

- Debug app exceeds its 116 KiB partition. An isolated copy of upstream main,
  using the same libraries/toolchain and only correcting the missing DSDL CMake
  include, occupies 137728 bytes; the port also exceeds the partition.
- VBBoot Debug exceeds 12 KiB: previous `c2336f5` uses 14068 bytes, new `2adc0bf`
  uses 14092 bytes. Release remains within the partition.
- `VBBoot/tests/test_bootloader_config.py`: 3 pass, 1 fails. The sample-point test
  expects 75%, but both old and new source use Seg1=55, Seg2=24, yielding 70%.
  Timing and the test were left unchanged; no CAN tuning is part of this port.

Recovery points are the branch `codex/vbdrive-before-main-sync-20260908` and the
stash named `VBDrive new parameters before main sync 2026-09-08`.
