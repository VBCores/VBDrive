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
register callback with host hardware/transport doubles. It checks all 40 reads and
types (all 40 registers shared, including `bootloader` and `cmd_errors`),
global readonly queries, every persistent
parameter's Serial write/EXIT rollback and Cyphal write,
readonly write rejection, CONFIG snapshots, RESET rollback, SAVE after an invalid
write, MIT/Servo commands, numeric validation, delayed EEPROM writes,
configuration before motor creation and the shared bootloader register. The EEPROM config size
and placements are checked (102 bytes; config 0, calibration 103).
Unified config writes/readback and preservation of bytes outside the config
are tested with a byte-addressed EEPROM double. Servo registers
are included in the write/rollback tests, with negative/non-finite gain rejection
and transient-form validation. The communication test also checks all four Servo
setpoint modes, invalid modes and wire equality with legacy specific_control.
It also exercises the actual FIFO/line consumer with split commands, multiple lines,
CR/LF/CRLF and overflow recovery, logging state rules and global STOP.
It does not simulate the FOC loop, physical encoders, UART DMA or CAN transport.

Calibration keeps the original blocking implementation. The interface test checks
that its Serial input is discarded rather than queued for subsequent execution.

## Servo regulators

Run `python3 tests/servo_control.py` for numerical tests of the actual FOC Servo
calculation and command setters, extracted unchanged and compiled with hardware
doubles. Checks cover position PID, velocity PI, measured-velocity D, target and gain
reset rules, invalid commands, saturation/unwinding, reduced limits, direction and
torque-to-current scaling. `parameter_interfaces.py` additionally verifies gain
application on SAVE, EEPROM reload after APPLY, and isolation of unsaved Serial
CONFIG values from Cyphal runtime writes and deferred persistence.

These host checks are not physical motor validation or FOC timing measurements.
The 2026-09-08 results below predate Servo PID/PI implementation.

## Shared Serial/Cyphal smoke, 2026-09-10

### Serial service fix — verified on hardware

The working implementation services Serial from PendSV once per millisecond,
at most one command per invocation, using DMA TX without waiting in the handler.
Busy TX prevents consumption of the next command or overwriting its response.
State logging uses the same service and TX buffer. Calibration, EEPROM writes,
driver I2C operations and reset remain in thread mode. A deferred operation
temporarily suppresses normal motor updates so thread mode can complete it;
the FOC algorithm, target laws and timer period are unchanged.

Host tests cover bounded dispatch, TX backpressure, deferred SAVE/APPLY,
calibration dispatch and driver enable. CubeMX-generated USART2 and PendSV
priorities are both 1. Release was flashed and verified: 109868 bytes Flash,
29392 bytes RAM. The full live register/CONFIG/EXIT/RESET/SAVE/APPLY test passed.

`serial_stress.py` exercises every Servo mode and MIT while requesting parameters
and receiving the state log. With the original 10 Hz log, at 20 requests/s, 420 queries passed with maximum
response 8.61 ms and maximum state-log gap 103.01 ms. At 100 requests/s, 2100
queries passed with maximum response 12.63 ms and maximum log gap 104.07 ms.
STOP and driver-disable acknowledgements passed; each run left the drive off.
No UART resets or debugger intervention were used during these stress runs.

The state log was then raised to 100 Hz (10 ms grid, without queuing stale samples).
The updated Release uses 109892 bytes Flash and 29392 bytes RAM and was flashed
and verified. With every Servo mode and MIT exercised at 115200 baud, all 2100
parameter queries at 100 requests/s passed alongside 2108 state samples:
measured state rate 100.03 Hz, maximum host-observed state gap 28.25 ms,
maximum response latency 17.17 ms. STOP, log_off and is_on:0 passed; readback
confirmed the drive off. Host parameter, MIT compatibility and Servo tests passed.
Report: `build/servo-validation/serial-service-state100hz-stress.json`.

Startup-log follow-up: VBBoot now waits for USART TC after `JAPP`, preserving
the final CRLF across the application jump. Config loading sends each formatted
fragment synchronously in thread mode through the existing 512-byte buffer;
runtime responses and state logging retain DMA. Host tests cover a multi-buffer
dump and its final Servo fields. The flashed Release uses 110156 bytes Flash
and 29392 bytes RAM; VBBoot uses 9360 bytes Flash. A raw reset capture confirms
`JAPP\r\nGot config_data`, `servo_tr_vel` and the final startup message:
`build/servo-validation/startup-log-fixed.bin`.
The live register/CONFIG/EXIT/RESET/SAVE/APPLY/logging/STOP regression passed;
APPLY also produced the complete startup dump. The drive was left off.
Report: `build/servo-validation/serial-startup-fix-registers.json`.

INFO/HELP follow-up (host- and hardware-tested): startup information now ends
with `See HELP for available commands`. INFO reuses that formatter without
reloading EEPROM or changing state; in CONFIG it displays staged settings.
HELP lists commands, parameter syntax and calibration restrictions. Both long
responses run through the deferred thread-mode Serial path. Host tests check
RUNNING/CONFIG dispatch, complete output beyond 512 bytes, all command names,
the final hint, and unchanged driver/state/EEPROM writes. Release builds with
111844 bytes Flash and 29392 bytes RAM. The image was flashed and verified after
explicit approval. Serial checks confirmed byte-for-byte equality between INFO
and the application startup message, the final hint, complete HELP, both commands
in CONFIG, and unchanged INFO after EXIT discarded staged gear changes. STOP and
driver-off readback passed. Captures: `build/servo-validation/startup-info-help.bin`
and `build/servo-validation/info-help.txt`.

Run on an explicitly authorized drive:

```sh
python3 tests/serial_stress.py --port /dev/cu.usbmodem1203 --motion --rate 100 --output build/servo-validation/serial-service-state100hz-stress.json
```

Reports are `build/servo-validation/serial-service-registers.json`,
`serial-service-stress-all-modes.json` and `serial-service-stress-100hz.json`.
An initial reuse of the older `unified_motion.py` stopped on its instantaneous
velocity-average assertion despite continuous replies and positive displacement;
that report remains `serial-service-motion.json` (failed). The new stress test
validates Serial delivery/latency, not mechanical tracking precision or FOC WCET.
Physical recalibration was not repeated; isolated calibration input and deferred
dispatch are covered by host tests.

### UART 115200 follow-up

USART2 was regenerated with STM32CubeMX at 115200 baud; only the baud setting
was retained from generation. VBBoot's hand-written UART initialization was
synchronized. Release application and bootloader were flashed together; EEPROM
was not erased. Host tests and the full live Serial parameter test passed.

The MIT -> STOP -> Servo motion test still lost timely replies. Read-only debugger
snapshots localized two delays in Serial servicing, not UART framing or overrun:

- At USART2 IRQ priority 3, IDLE was set and IRQ 38 remained pending. RX DMA had
  received 24 bytes; HAL reported ErrorCode=0. One snapshot also showed hardware
  TX complete while HAL gState was still BUSY_TX, awaiting the UART TC handler.
- Temporarily raising only USART2 priority to 1 in NVIC cleared its pending bit
  and rearmed RX DMA. However, the foreground Serial FIFO still held 77 bytes
  (head=106, tail=29), and commands were not promptly dispatched.
- Across the zero-Servo snapshots, the priority-0 millisecond timer advanced by
  141453 ms while the priority-15 SysTick advanced by only 17 ms. TIM4 was both
  active and pending. Debugger stops perturb scheduling, but cannot account for
  this large difference. An isolated FOC maximum above 4000 cycles is not used
  as proof of the cause; the IRQ/FIFO/timer observations are the relevant evidence.

The temporary priority change was removed by reset. No FOC or permanent interrupt
priority changes were made. Increasing baud alone does not resolve delayed Serial
service. Reports: `build/servo-validation/serial-115200-registers.json`,
`serial-115200-motion.json`, `serial-115200-diag-1.txt` through `-4.txt`, and
`serial-115200-irq1.json`.

### Initial 19200-baud smoke

Release was flashed and verified on ST-Link `0672FF544983555067215514`:
109420 bytes Flash, 29376 bytes RAM. Host parameter, Servo numerical and MIT/legacy
wire tests pass. Calibration uses its unchanged blocking algorithm, without input
handling or cancellation; host tests cover discarding queued and partial Serial
commands at the calibration boundary. No physical recalibration was performed.

The full live Serial test passed all 40 registers, readonly rejection, persistent
Servo settings, CONFIG/EXIT/RESET/SAVE/APPLY, removed commands, logging and STOP.
Fragmented/batched input and overlong-line recovery passed on hardware. All four
Servo modes acknowledged commands without enabling the disabled motor. Serial
`bootloader:1` entered VBBoot; an addressed reset returned to the application.
Cyphal reads confirmed the same revision and five persistent Servo gains.

MIT movement and STOP worked. The combined motion test then lost Serial responses
before its Servo step; an ST-Link snapshot found execution inside the FOC timer
interrupt. After reset, a separate Servo velocity command produced movement and
STOP/log_off/is_on:0 succeeded, but telemetry/command responses were delayed.
This is not a passing real-time or sustained-motion qualification. The existing
FOC timing issue was not changed or diagnosed conclusively in this task.

Reports: `build/servo-validation/unified-serial.json` (pass),
`unified-motion.json` (failed combined run), `unified-servo.json`,
`unified-servo-motion.json` and `unified-final.json`. Motor left disabled.

## Servo hardware smoke, 2026-09-09

### Shared PID refactor

Servo now uses two actual libvoltbro `PIDRegulator` instances, not FOC-owned
gain/integral fields. `regulation_with_derivative` adds measured-derivative D,
dynamic output/I clamping and conditional integration without changing existing
`regulation` overloads used by the current loops. `reset` clears all PID state.
Host tests include the real PID header, exercise its old overloads as well as
Servo numerical/reset behavior, and verify NAN default resolution and explicit
zero gains. All parameter and MIT/legacy wire regressions pass.

Defaults live in `VBDriveDefaults`; stored Servo floats start as NAN, and the
integer trajectory selector uses 0 as an unset sentinel. Serial/Cyphal resolve
these to the same effective values used to initialize the motor.

Release with profiling was rebuilt, flashed and verified: Flash 107588 bytes,
RAM 28656 bytes. Serial CONFIG/RESET/EXIT read all seven effective defaults;
Cyphal confirmed the five persisted gains. Four 2-second motor checks passed:
PI mean speeds +0.04579/-0.04721 rad/s at +/-0.05 rad/s, PID final errors
0.00109/0.00313 rad at +/-0.03 rad steps. Telemetry counts were only 101-132
per run; this is motion validation, not a timing/communication-rate guarantee.
No new FOC deadline qualification was performed; the earlier timing issue remains.
Artifacts: `build/servo-validation/pid-shared-*.json`. Motor left disabled with
the selected gains persisted. The final API-name cleanup produced the identical
flashed HEX SHA-256 `6190864b8d5be25ba3f52625975796be1f046a2c40c9c2ad7df4dc373eae8f5c`.

### Default-gain follow-up

Selected starting defaults: position PID **150 / 200 / 10**, velocity PI
**30 / 60**. Tested on the same M4310 at gear=36, kt=0.5, max_i=0.3 A and
max_tq=5 Nm, using 2-second commands in both directions on Release.

| Check | Positive command | Negative command |
| --- | --- | --- |
| Velocity command | +0.05 rad/s | -0.05 rad/s |
| Mean velocity, second half, PI 30/30 | +0.0326 rad/s | -0.0388 rad/s |
| Mean velocity, second half, selected PI 30/60 | +0.0445 rad/s | -0.0469 rad/s |
| Position step | +0.03 rad | -0.03 rad |
| Final absolute position error, PID 150/200/10 | 0.00065 rad | 0.00542 rad |

Selected PI peak speeds were 0.108 and 0.095 rad/s: low-speed ripple remains,
so this is practical starting tuning, not precision tracking qualification.
The motion checks explicitly required final position error <0.01 rad or
second-half mean velocity error <0.025 rad/s, in addition to finite telemetry
and the existing motion envelope. All six comparison runs passed.
Reports are `build/servo-validation/defaults-*.json`.
The default constructor/RESET values are covered by `parameter_interfaces.py`;
numerical Servo and MIT wire regressions also pass. Release with profiling uses
107852 bytes Flash and 28688 bytes RAM. Existing EEPROM values are not silently
replaced by these new defaults. The FOC timing limitation below remains open.
The rebuilt Release was flashed and verified on the addressed G431. Serial
CONFIG/RESET exposed all five new defaults; EXIT discarded the other reset
settings. Both Serial and Cyphal confirmed the selected gains after flashing,
is_on=0 and unchanged current/torque limits; Cyphal cmd_errors=0.
Final Serial transcript: `build/servo-validation/defaults-final-readback.json`.
The motor is left disabled with the selected gains persisted, superseding the
zero-gain final state of the initial validation below. No commit was created;
commit libvoltbro changes before updating its gitlink in the VBDrive commit.

### Initial implementation validation

- Target: STM32G431, ST-Link `0672FF544983555067215514`, Serial
  `/dev/cu.usbmodem1203` at 19200, Cyphal node 11 through VM `vcan1.3`.
  Only this VBDrive was flashed; EthernetCAN firmware/configuration was not changed.
- Backed up Flash and EEPROM, erased the incompatible EEPROM layout with user
  authorization, provisioned fresh configuration and recalibrated the motor.
  Correct backups are `build/servo-validation/vbdrive-flash-before.bin` and
  `vbdrive-eeprom-before.bin`; the latter was read twice and compared byte-for-byte.
- Both builds fit: RelWithDebInfo Flash/RAM 105480/28824 bytes;
  Release with `VBDRIVE_FOC_PROFILE=ON` 107788/28688 bytes. The latter was flashed
  and verified. Reported revision `603b3ac5bfa3c0e0` identifies the base commit,
  not these uncommitted changes.
- Release passed all 38 Serial reads, readonly checks, CONFIG/EXIT/RESET/SAVE/APPLY
  and TEST/logging smoke. Cyphal read/write of all five Servo gains passed;
  zero values written through Cyphal survived APPLY/reboot and Serial readback.
- At max_i=0.3 A, max_tq=5 Nm, gear=36 and kt=0.5, velocity PI (P=30, I=30)
  tracked a 0.05 rad/s command for 2 s: mean velocity in the second half
  0.0401 rad/s, displacement 0.05418 rad, 1454 telemetry samples.
- Position PID (P=150, I=200, D=10) received a 0.03 rad step for 2 s:
  displacement 0.02876 rad, final error 0.00124 rad, 1116 samples.
  These are bounded functional smoke tests, not production gain tuning.
- **Timing remains a known failure:** sampled complete `main_callback` cost at
  160 MHz peaked at 6329 cycles (39.56 us) on RelWithDebInfo. Release improved
  communication and typical samples, but still recorded peaks of 5683 cycles
  (35.52 us) after PI and 5793 cycles (36.21 us) after PID, exceeding 25 us.
  The profiler samples every 256 calls; these are observed maxima, not WCET
  bounds. ISR entry/exit and profiling bookkeeping are not included. No claim
  of a guaranteed 40 kHz deadline is made; timing optimization remains separate.
- Host numerical, parameter-interface and MIT/legacy wire tests all pass, as do
  root/libvoltbro `git diff --check`. No new subscriptions or runtime heap use
  were introduced. DRV_FAULT remains outside this check.
- Final state confirmed by Serial after reboot: all five Servo gains zero,
  max_i=0.3 A, max_tq=5 Nm, is_on=0. Cyphal cmd_errors was zero after movement.
  GDB server stopped. No commits or pushes made.
- Local artifacts: `build/servo-validation/release-serial.json`,
  `vbdrive-release-velocity.json`, `vbdrive-release-position.json`, and
  `release-final-readback.json`. These build artifacts are not tracked.

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
