# VBDrive specifications

This directory contains the current VBDrive behavioral baseline. The firmware
implementation is the source of truth when capturing existing behavior; README
and baseline specs are maintained to describe it accurately. Proposed behavior
changes live in `changes/` until they are implemented, verified and synchronized
into `specs/`.

## Capabilities

| Specification | Contract |
| --- | --- |
| [Control timing](specs/control-timing/spec.md) | 40 kHz control, 5 kHz references, 8 kHz integral, bounded RX/TX, State cadence and timing qualification |
| [Register interface](specs/register-interface/spec.md) | All 50 registers, types/defaults/access, Serial/Cyphal formats, activation and persistence |
| [Control modes](specs/control-modes/spec.md) | Seven Servo modes, MIT, control laws, limits, command identity and rejection |
| [Coordinates and units](specs/coordinates-and-units/spec.md) | Rotor/electrical/output/user frames, ang_dir, ang_off, gear, kt and telemetry units |
| [Drive lifecycle](specs/drive-lifecycle/spec.md) | INIT/RUNNING/CONFIG/NOT_CALIBRATED/CALIBRATING, ON/OFF, STOP, calibration and bootloader |
| [FOC control](specs/foc-control/spec.md) | Observer, current regulation, clipping, modulation, bus-current estimate and disabled sensing |
| [Temperature protection](specs/temperature-protection/spec.md) | Sensor validity, stator hysteresis, MCU limits, neutral recovery and Heartbeat health/status |
| [Trajectory generation](specs/trajectory-generation/spec.md) | FILTER/POLY/RAMP equations, initialization, continuous retargeting and configuration |

Requirements deliberately focus on firmware behavior. They do not freeze every
internal class or peripheral implementation choice. Product behavior implemented
in a submodule is covered here; changes to a reusable library API must identify
its owning repository and affected consumers rather than create competing copies
of its contract.

## Working on a change

1. Read the affected specs and their implementation/test references below. Use
   `openspec-explore` when the desired behavior or approach needs investigation.
2. Use `openspec-propose` to create a focused change. Scaffold new changes through
   `openspec new change <name>`, then use its status/instructions. Describe new or
   changed requirements as deltas; do not silently rewrite accepted behavior.
3. Use `openspec-apply-change` for implementation. Update the plan when the
   agreed approach changes and keep README aligned with the resulting code.
4. Use `openspec-verify-change` and the relevant checks from
   [tests/README.md](../tests/README.md). Hardware timing and motor behavior need
   appropriate physical evidence, not an inference from host tests.
5. Archive completed changes with `openspec-archive-change`, merging accepted
   deltas into the main specs. Do not mark unperformed verification complete.

A pure documentation/tooling/refactoring change with no behavioral delta may use
`skip_specs: true`. Technical design is needed when the schema's conditions apply,
particularly for timing, interrupt/state publication or persistence changes.
Use the existing global OpenSpec skills; this root does not install duplicate
project-local skill copies.

```sh
openspec list
openspec list --specs
openspec validate --all --strict --no-interactive
```

`validate` checks specification structure. It neither executes tests nor proves
firmware compliance. Scope future checks to the affected behavior; retain the
hardware target, authorization and settings-restoration requirements already
specified in AGENTS.md, tests/README.md and the active user session.

## Implementation and verification map

All paths below are relative to the repository root. Test commands run as
`python3 tests/<name>.py`; setup requirements are in tests/README.md.

| Area | Implementation | Host checks |
| --- | --- | --- |
| Timing | `Core/Src/tim.c`, `Core/Src/stm32g4xx_it.c`, `App/app.cpp`, `App/communications/{serial,cyphal}/setup.hpp`, `Drivers/libvoltbro/voltbro/motors/bldc/foc/foc.cpp` | `pwm_foc_sync`, `millis_clock`, `control_counters`, `cyphal_tx`, `servo_control` |
| Registers and persistence | `App/config/config.{hpp,cpp}`, `App/communications/{serial,cyphal}/interface.hpp`, `Drivers/libvoltbro/voltbro/config/`, `VBBoot` | `parameter_interfaces`, `refactor_contracts`, `mit_control` |
| Motion modes and coordinates | `Drivers/libvoltbro/voltbro/motors/bldc/{bldc.h,foc/foc.hpp,foc/foc.cpp,vbdrive/vbdrive.hpp}`, `Drivers/libvoltbro/voltbro/math/regulators/pid.hpp` | `servo_control`, `rated_limits`, `mit_control`, `kalman_filter` |
| Lifecycle | `App/state_manager/state_manager.{h,cpp}`, `App/app.cpp`, `Drivers/libvoltbro/voltbro/motors/bldc/vbdrive/vbdrive.hpp` | `parameter_interfaces`, `servo_control`, `cyphal_arena` |
| FOC and sensing | `Drivers/libvoltbro/voltbro/motors/bldc/{foc,vbdrive}/`, `Drivers/libvoltbro/voltbro/math/regulators/pid.hpp` | `kalman_filter`, `bus_current`, `rated_limits`, `pwm_foc_sync` |
| Temperature protection | `App/state_manager/`, `App/communications/cyphal/setup.hpp`, `Drivers/libvoltbro/voltbro/motors/bldc/vbdrive/` | `temperature_conversion`, `thermal_protection`, `parameter_interfaces` |
| Generators | `Drivers/libvoltbro/voltbro/motors/trajectories/`, `Drivers/libvoltbro/voltbro/motors/bldc/foc/foc.{hpp,cpp}` | `servo_input`, `servo_control` |

These tests cover selected production paths with host substitutes where needed.
The table is a navigation and regression guide, not a claim that every scenario
has an automated or physical test. Nominal rates and divider ratios are distinct
from measured control throughput, handler jitter and delivered State messages.
