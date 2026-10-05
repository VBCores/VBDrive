#!/usr/bin/env python3
"""Numerical host checks of the actual Servo calculation and command setters; no hardware."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "Drivers/libvoltbro/voltbro/motors/bldc"
bldc = (BASE / "bldc.h").read_text()
foc = (BASE / "foc/foc.hpp").read_text()
implementation = (BASE / "foc/foc.cpp").read_text()


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


source = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <utility>
#include <limits>
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/profiling.hpp"
unsigned primask = 0;
void (*irq_hook)() = nullptr;
unsigned __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1; }
void __set_PRIMASK(unsigned value) { primask = value; if (!value && irq_hook) { auto hook=irq_hook; irq_hook=nullptr; hook(); } }
'''
utils = (ROOT / "Drivers/libvoltbro/voltbro/utils.hpp").read_text()
source += utils[utils.index("#define CRITICAL_SECTION"):utils.index("// TODO: add optional warning")]
source += bldc[bldc.index("enum class SetPointType"):bldc.index("enum class DrivePhase")]
source += foc[foc.index("struct FOCTarget"):foc.index("struct FiltersConfig")]
source += '#include "voltbro/motors/bldc/foc/servo_control.hpp"\n'
source += r'''
struct DriveInfo {
    float torque_const=.5f, max_torque=30;
    struct { unsigned gear_ratio=2; } common;
};
struct RuntimeConfig {
    float user_current_limit=30, current_limit=30, user_torque_limit=30;
    float user_speed_limit=NAN, user_position_lower_limit=NAN, user_position_upper_limit=NAN;
    int user_angle_direction=1;
    float user_angle_offset=0;
};
struct BLDCController {
    virtual int stop() { return 0; }
    virtual int start() { return 0; }
    DriveInfo drive_info;
    RuntimeConfig drive_runtime_config;
    SetPointType point_type=SetPointType::VOLTAGE;
    float target=0, shaft_angle=0, shaft_velocity=0;
'''
source += bldc[bldc.index("    FORCE_INLINE bool is_symmetric_limit_set"):bldc.index("    const DriveInfo drive_info;")]
for name in ("virtual void reset_control()", "void set_point(",
             "FORCE_INLINE virtual bool set_angle_point", "FORCE_INLINE virtual bool set_velocity_point",
             "FORCE_INLINE virtual bool set_torque_point", "FORCE_INLINE virtual bool set_voltage_point",
             "FORCE_INLINE float get_angle()", "FORCE_INLINE float get_velocity()"):
    source += function(bldc, name) + "\n"
source += r'''
};
struct FOC : BLDCController {
    float T=.000025f;
    ServoInputConfig servo_input_config;
    std::optional<ServoCommand> servo_command;
    std::optional<ServoTrajectoryStorage> servo_traj_generator;
    uint32_t control_tick=0;
    uint8_t servo_reference_ticks=0, servo_integral_ticks=0;
    bool servo_reference_initialize=false;
    FOCTarget foc_target;
    PIDRegulator servo_pos_reg, servo_vel_reg;
    float servo_torque();
    float servo_period() {
        float result=0;
        for(int i=0;i<5;++i) { ++control_tick; result=servo_torque(); }
        return result;
    }
'''
for name in ("void reset_control()", "bool set_foc_point(", "void reset_servo_input()", "bool set_servo_input_config(", "bool set_servo_command(", "PIDConfig get_servo_config(", "void update_servo_config("):
    source += function(foc, name) + "\n"
source += "};\n" + function(implementation, "float FOC::servo_torque()")
source += r'''
using HAL_StatusTypeDef = int;
constexpr int HAL_OK=0;
unsigned HAL_GetTick() { return 100; }
#define __HAL_TIM_MOE_DISABLE(timer) bridge_enabled=false
#define __HAL_TIM_MOE_ENABLE(timer) bridge_enabled=true
struct VBDrive : FOC {
    bool _is_on=false, bridge_enabled=false;
    unsigned bootstrap_charge_deadline_ms=0, bootstrap_charge_time_ms=2;
    struct Gate {
        int request_standby() { return HAL_OK; }
        int wake() { return HAL_OK; }
        int clear_faults() { return HAL_OK; }
    } gate_driver;
    void force_bootstrap_charge() {}
    void quit_stall() {}
'''
vbdrive = (BASE / "vbdrive/vbdrive.hpp").read_text()
source += function(vbdrive, "HAL_StatusTypeDef stop() override")
source += function(vbdrive, "HAL_StatusTypeDef start() override")
source += "};\n"
current_conversion = next(line.strip() for line in implementation.splitlines()
                          if "i_q_set = controller_response" in line)
source += r'''
float servo_current(FOC& motor) {
    float i_q_set;
    const float controller_response=motor.servo_period();
    const auto& drive_info=motor.drive_info;
    auto get_direction_multiplier=[&] {return motor.get_direction_multiplier();};
''' + current_conversion + r'''
    return i_q_set;
}
'''
mit_start = implementation.index("i_q_set =", implementation.index("if (point_type == SetPointType::UNIVERSAL)"))
mit_expression = implementation[mit_start:implementation.index(";", mit_start) + 1]
source += r'''
float mit_current(FOC& motor) {
    const auto& foc_target=motor.foc_target;
    const auto& drive_info=motor.drive_info;
    const float gear_ratio_f=drive_info.common.gear_ratio;
    auto get_angle=[&] {return motor.get_angle();};
    auto get_velocity=[&] {return motor.get_velocity();};
    auto get_direction_multiplier=[&] {return motor.get_direction_multiplier();};
    float i_q_set;
''' + mit_expression + r'''
    return i_q_set;
}
'''
source += r'''
float integral(const PIDRegulator& regulator) { return regulator.get_integral_error() * regulator.get_config().ki; }
void near(float actual, float expected, float tolerance=1e-5f) { assert(std::fabs(actual-expected)<tolerance); }
FOC* interrupted_motor=nullptr;
void foc_interrupt() { ++interrupted_motor->control_tick; interrupted_motor->servo_torque(); }
int main() {
    // Legacy PID overloads retain their behavior, including dynamic-limit I gating.
    PIDRegulator legacy({.kp=2, .ki=4, .kd=3});
    near(legacy.regulation(1.f,.1f),32.4f);
    near(legacy.regulation(1.f,.1f),2.8f);
    legacy.reset();
    near(legacy.regulation(1.f,.1f,1.f),32.4f);
    near(legacy.get_integral_error(),0);
    legacy.reset();
    near(legacy.regulation(1.f,.1f,-1.f,1.f),32.4f);
    near(legacy.get_integral_error(),0);
    legacy.update_config(PIDConfig{.kp=2,.ki=4,.tolerance=.2f});
    near(legacy.regulation(1.f,.1f,-1.f,1.f,0),2.4f); // Integer bool remains unambiguous.
    near(legacy.regulation(.1f,.1f,true),0);
    near(legacy.get_integral_error(),0);
    FOC separate;
    separate.set_angle_point(1); separate.shaft_velocity=.25f;
    separate.update_servo_config(SetPointType::POSITION,{.kp=2});
    near(separate.servo_period(),2);
    separate.update_servo_config(SetPointType::POSITION,{.ki=4});
    near(separate.servo_period(),.0005f); near(separate.servo_period(),.001f);
    separate.update_servo_config(SetPointType::POSITION,{.kd=3});
    near(separate.servo_period(),-.75f);
    separate.set_angle_point(2); near(separate.servo_period(),-.75f);
    separate.set_velocity_point(1);
    separate.update_servo_config(SetPointType::VELOCITY,{.kp=2});
    near(separate.servo_period(),1.5f);
    separate.update_servo_config(SetPointType::VELOCITY,{.ki=4});
    near(separate.servo_period(),.000375f); near(separate.servo_period(),.00075f);
    FOC motor;
    motor.update_servo_config(SetPointType::POSITION, {.kp=2, .ki=4, .kd=3});
    assert(motor.set_angle_point(2));
    motor.shaft_angle=1; motor.shaft_velocity=.5f;
    near(motor.servo_period(), .5005f);
    near(integral(motor.servo_pos_reg), .0005f);
    assert(motor.set_angle_point(3));
    near(integral(motor.servo_pos_reg), .0005f); // Same-mode target preserves I.
    near(motor.servo_period(), 2.5015f); // No D kick from a target step.
    auto gains=motor.get_servo_config(SetPointType::POSITION);
    primask=1; motor.update_servo_config(SetPointType::POSITION,gains);
    assert(primask==1); near(integral(motor.servo_pos_reg),.0015f);
    primask=0; gains.kp=1; motor.update_servo_config(SetPointType::POSITION,gains);
    assert(primask==0); near(integral(motor.servo_pos_reg),0); near(motor.target,3);

    motor.update_servo_config(SetPointType::VELOCITY,{.kp=2,.ki=4});
    assert(motor.set_velocity_point(1));
    near(integral(motor.servo_pos_reg),0);
    near(motor.servo_period(),1.00025f);
    const float old_integral=integral(motor.servo_vel_reg);
    for (float invalid : {NAN,INFINITY,-INFINITY}) {
        assert(!motor.set_velocity_point(invalid)); assert(!motor.set_angle_point(invalid));
        assert(!motor.set_torque_point(invalid)); assert(!motor.set_voltage_point(invalid));
        assert(!motor.set_foc_point({.angle=invalid}));
        assert(!motor.set_foc_point({.angle_kp=invalid}));
        assert(motor.point_type==SetPointType::VELOCITY); near(motor.target,1);
        near(integral(motor.servo_vel_reg),old_integral);
    }
    motor.drive_runtime_config.user_speed_limit=.5f;
    assert(!motor.set_velocity_point(1)); assert(motor.set_angle_point(1));
    motor.update_servo_config(SetPointType::POSITION,{});
    near(motor.servo_period(),0);

    // Both saturation directions, unwind while saturated, and a reduced current limit.
    motor.update_servo_config(SetPointType::POSITION,{.kp=10,.ki=4});
    motor.drive_runtime_config.user_torque_limit=2;
    motor.shaft_angle=0; motor.shaft_velocity=0;
    for (float sign : {-1.f,1.f}) {
        motor.servo_pos_reg.set_integral_error((0) / motor.servo_pos_reg.get_config().ki); motor.set_angle_point(sign);
        near(motor.servo_period(),sign); near(integral(motor.servo_pos_reg),0);
    }
    motor.servo_pos_reg.set_integral_error((2) / motor.servo_pos_reg.get_config().ki); motor.set_angle_point(-.01f);
    motor.servo_period(); assert(integral(motor.servo_pos_reg)<2);
    motor.drive_runtime_config.current_limit=.25f;
    motor.servo_period(); assert(std::fabs(integral(motor.servo_pos_reg))<=.125f);
    motor.drive_runtime_config.current_limit=30;
    motor.update_servo_config(SetPointType::POSITION,{.ki=4,.kd=1});
    motor.servo_pos_reg.set_integral_error((2) / motor.servo_pos_reg.get_config().ki); motor.shaft_velocity=-4;
    near(motor.servo_period(),1); assert(integral(motor.servo_pos_reg)<2); // Unwind despite saturation.
    motor.set_voltage_point(0); near(integral(motor.servo_pos_reg),0); near(integral(motor.servo_vel_reg),0);
    motor.set_foc_point({.torque=1}); motor.set_angle_point(0); near(motor.foc_target.torque,0);

    // Actual torque-to-Iq expression, corrected coordinates, direction applied once.
    motor.update_servo_config(SetPointType::POSITION,{.kp=1});
    motor.drive_runtime_config.user_torque_limit=30;
    motor.drive_info.torque_const=.25f; motor.drive_info.common.gear_ratio=4;
    for (int direction : {-1,1}) {
        motor.drive_runtime_config.user_angle_direction=direction;
        motor.shaft_angle=.5f*direction;
        motor.drive_runtime_config.user_angle_offset=.25f;
        motor.set_angle_point(1);
        near(servo_current(motor),1.f*direction);
        motor.drive_info.common.gear_ratio=8;
        near(servo_current(motor),1.f*direction);
        motor.drive_info.common.gear_ratio=4;
    }
    motor.drive_info.torque_const=1;
    motor.drive_info.common.gear_ratio=10;
    motor.drive_runtime_config.user_angle_offset=0;
    motor.shaft_angle=0;
    motor.shaft_velocity=0;
    for (int direction : {-1,1}) {
        motor.drive_runtime_config.user_angle_direction=direction;
        assert(motor.set_foc_point({.angle=.1f, .angle_kp=2.f}));
        near(mit_current(motor), .2f*direction);
        assert(motor.set_foc_point({.velocity=.05f, .velocity_kp=4.f}));
        near(mit_current(motor), .2f*direction);
        assert(motor.set_foc_point({.torque=.2f}));
        near(mit_current(motor), .02f*direction);
        assert(motor.set_foc_point({.torque=.2f, .angle=.1f, .velocity=.05f,
                                    .angle_kp=2.f, .velocity_kp=4.f}));
        near(mit_current(motor), .42f*direction);
    }
    // With zero I and no MIT feedforward, equal gains must request equal Iq.
    for (unsigned gear : {1U, 10U, 36U}) {
        FOC equivalent;
        equivalent.drive_info.torque_const=1;
        equivalent.drive_info.common.gear_ratio=gear;
        equivalent.shaft_angle=.1f; equivalent.shaft_velocity=.02f;
        equivalent.update_servo_config(SetPointType::POSITION,{.kp=2,.kd=.5f});
        assert(equivalent.set_angle_point(.2f));
        const float servo_position_current=servo_current(equivalent);
        assert(equivalent.set_foc_point({.angle=.2f, .velocity=0, .torque=0,
                                         .angle_kp=2, .velocity_kp=.5f}));
        near(servo_position_current,mit_current(equivalent));
        equivalent.update_servo_config(SetPointType::VELOCITY,{.kp=.5f,.ki=0});
        assert(equivalent.set_velocity_point(.1f));
        const float servo_velocity_current=servo_current(equivalent);
        assert(equivalent.set_foc_point({.velocity=.1f, .velocity_kp=.5f}));
        near(servo_velocity_current,mit_current(equivalent));
    }
    VBDrive device;
    device.update_servo_config(SetPointType::POSITION,{.kp=1,.ki=1});
    device.start(); device.set_angle_point(1); device.servo_period();
    assert(integral(device.servo_pos_reg)>0);
    device.stop(); assert(!device._is_on && !device.bridge_enabled);
    near(integral(device.servo_pos_reg),0); near(device.target,0);
    device.set_angle_point(2); // A target received while off must not resume on enable.
    device.start(); assert(device._is_on && device.bridge_enabled);
    assert(device.point_type==SetPointType::VOLTAGE); near(device.target,0);
    near(integral(device.servo_pos_reg),0);
    FOC generated;
    ServoInputConfig input_config; input_config.input_bandwidth=10;
    assert(generated.set_servo_input_config(input_config));
    generated.shaft_angle=.5f;
    assert(generated.set_servo_command(ServoControlType::POSITION_FILTER,1,true,7));
    generated.servo_period();
    float reference=std::get<FilterTrajectory>(*generated.servo_traj_generator).reference;
    assert(generated.set_servo_command(ServoControlType::POSITION_FILTER,1,true,7));
    assert(std::get<FilterTrajectory>(*generated.servo_traj_generator).reference==reference);
    assert(!generated.set_servo_command(ServoControlType::POSITION_FILTER,2,true,7));
    assert(generated.set_servo_command(ServoControlType::POSITION_FILTER,2,true,8));
    assert(std::get<FilterTrajectory>(*generated.servo_traj_generator).reference==reference);
    input_config.input_bandwidth=0;
    assert(!generated.set_servo_input_config(input_config));
    generated.reset_servo_input();
    assert(generated.set_servo_input_config(input_config));
    assert(!generated.servo_command && !generated.servo_traj_generator);
    generated.servo_input_config={.input_bandwidth=10,.velocity_limit=1,
        .acceleration_limit=2,.deceleration_limit=2,.velocity_ramp_rate=2};
    assert(generated.set_servo_command(POSITION_POLY,1,true,255));
    const auto epoch_elapsed=std::get<PolyTrajectory>(*generated.servo_traj_generator).elapsed;
    generated.control_tick+=40;
    assert(generated.set_servo_command(POSITION_POLY,1,true,255));
    assert(std::get<PolyTrajectory>(*generated.servo_traj_generator).elapsed==epoch_elapsed);
    assert(!generated.set_servo_command(POSITION_POLY,2,true,255));
    assert(generated.servo_command->value==1 && generated.servo_command->index==255);
    assert(generated.set_servo_command(POSITION_POLY,1,true,0));
    near(std::get<PolyTrajectory>(*generated.servo_traj_generator).elapsed, generated.T, 1e-8f);
    generated.control_tick+=40;
    assert(generated.set_servo_command(POSITION_POLY,1));
    assert(!generated.servo_command->has_index);
    near(std::get<PolyTrajectory>(*generated.servo_traj_generator).elapsed, generated.T, 1e-8f);
    assert(generated.set_servo_command(POSITION_DIRECT,1));
    assert(!generated.servo_traj_generator && generated.servo_command);
    assert(generated.set_foc_point({}));
    assert(!generated.servo_traj_generator && !generated.servo_command);
    device.servo_input_config.velocity_ramp_rate=2;
    assert(device.set_servo_command(VELOCITY_RAMP,1,true,3));
    device.stop();
    assert(!device.servo_traj_generator && !device.servo_command);
    PIDRegulator live_limit({.ki=2});
    live_limit.set_integral_error(1);
    near(live_limit.regulation_with_derivative(0,.000025f,-.1f,.1f,0,false),.1f);
    near(live_limit.get_integral_error(),.05f);
    near(live_limit.regulation_with_derivative(0,.000025f,-10,10,0,false),.1f);

    // Five changing errors are integrated exactly once; P reacts every tick.
    FOC divided;
    divided.update_servo_config(SetPointType::POSITION,{.kp=1,.ki=2});
    divided.set_angle_point(1);
    float sum=0;
    for(int i=1;i<=5;++i) {
        divided.target=float(i); sum+=i;
        const float response=divided.servo_torque();
        near(response,float(i)+(i==5 ? 2*sum*divided.T : 0));
    }
    near(integral(divided.servo_pos_reg),2*sum*divided.T);
    divided.set_foc_point({}); assert(divided.servo_integral_ticks==0);

    // 5 kHz reference is held over exactly eight 40 kHz calls.
    FOC cadence;
    cadence.servo_input_config.velocity_ramp_rate=2;
    assert(cadence.set_servo_command(ServoControlType::VELOCITY_RAMP,1));
    for(int i=0;i<16;++i) {
        ++cadence.control_tick; cadence.servo_torque();
        near(cadence.target,i<8 ? .0004f : .0008f);
    }
    // Interrupt just after the snapshot must not be overwritten by command commit.
    for(uint8_t type : {ServoControlType::VELOCITY_RAMP,ServoControlType::POSITION_FILTER}) {
        FOC racing; racing.servo_input_config={.input_bandwidth=20,.velocity_ramp_rate=2};
        assert(racing.set_servo_command(type,1));
        interrupted_motor=&racing; irq_hook=foc_interrupt;
        assert(racing.set_servo_command(type,2));
        assert(std::visit([](const auto& g){return g.reference;},*racing.servo_traj_generator)>0);
        assert(racing.servo_reference_ticks==7); // ISR's phase survives too.
        const float previous_ref=std::visit([](const auto& g){return g.reference;},*racing.servo_traj_generator);
        racing.servo_reference_ticks=0;
        irq_hook=foc_interrupt;
        assert(racing.set_servo_input_config({.input_bandwidth=30,.velocity_ramp_rate=3}));
        assert(std::visit([](const auto& g){return g.reference;},*racing.servo_traj_generator)>previous_ref);
        assert(racing.servo_reference_ticks==7);

    }
    // Planning duration is included in the first future POLY reference.
    FOC aged;
    aged.servo_input_config={.velocity_limit=1,.acceleration_limit=2,.deceleration_limit=2};
    interrupted_motor=&aged; irq_hook=[] { interrupted_motor->control_tick+=40; };
    assert(aged.set_servo_command(ServoControlType::POSITION_POLY,1));
    ++aged.control_tick; aged.servo_torque();
    near(std::get<PolyTrajectory>(*aged.servo_traj_generator).elapsed,.001225f,1e-8f);
    // Replanning during a held reference preserves its phase and first-step horizon.
    for(int phase=1;phase<=7;++phase) {
        FOC held; held.servo_input_config=aged.servo_input_config;
        assert(held.set_servo_command(POSITION_POLY,1));
        for(int i=0;i<phase;++i) { ++held.control_tick; held.servo_torque(); }
        const auto epoch=held.control_tick;
        assert(held.set_servo_command(POSITION_POLY,2));
        do { ++held.control_tick; held.servo_torque(); } while(held.servo_reference_ticks!=7);
        near(std::get<PolyTrajectory>(*held.servo_traj_generator).elapsed,
             float(held.control_tick-epoch)*held.T+8*held.T,1e-8f);
    }
    FOC wrapped; wrapped.servo_input_config=aged.servo_input_config;
    wrapped.control_tick=UINT32_MAX-3;
    interrupted_motor=&wrapped; irq_hook=[] { interrupted_motor->control_tick+=5; };
    assert(wrapped.set_servo_command(POSITION_POLY,1));
    ++wrapped.control_tick; wrapped.servo_torque();
    near(std::get<PolyTrajectory>(*wrapped.servo_traj_generator).elapsed,14*wrapped.T,1e-8f);
    const auto config_epoch=wrapped.control_tick;
    interrupted_motor=&wrapped; irq_hook=foc_interrupt;
    assert(wrapped.set_servo_input_config(wrapped.servo_input_config));
    do { ++wrapped.control_tick; wrapped.servo_torque(); } while(wrapped.servo_reference_ticks!=7);
    near(std::get<PolyTrajectory>(*wrapped.servo_traj_generator).elapsed,
         (wrapped.control_tick-config_epoch+8)*wrapped.T,1e-8f);
    puts("Servo numerical and command-state checks passed");
}
'''
with tempfile.TemporaryDirectory(prefix="vbdrive-servo-") as temporary:
    path = Path(temporary)
    (path / "test.cpp").write_text(source)
    subprocess.run(["c++", "-std=c++20", "-Wall", "-Wextra", "-I"+str(ROOT / "Drivers/libvoltbro"), str(path / "test.cpp"), "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
