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
unsigned primask = 0;
unsigned __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1; }
void __set_PRIMASK(unsigned value) { primask = value; }
'''
utils = (ROOT / "Drivers/libvoltbro/voltbro/utils.hpp").read_text()
source += utils[utils.index("#define CRITICAL_SECTION"):utils.index("// TODO: add optional warning")]
source += bldc[bldc.index("enum class SetPointType"):bldc.index("enum class DrivePhase")]
source += foc[foc.index("struct FOCTarget"):foc.index("struct FiltersConfig")]
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
    FOCTarget foc_target;
    PIDRegulator servo_pos_reg, servo_vel_reg;
    float servo_torque();
'''
for name in ("void reset_control()", "bool set_foc_point(", "PIDConfig get_servo_config(", "void update_servo_config("):
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
    const float controller_response=motor.servo_torque();
    const auto& drive_info=motor.drive_info;
    const float gear_ratio_f=drive_info.common.gear_ratio;
    auto get_direction_multiplier=[&] {return motor.get_direction_multiplier();};
''' + current_conversion + r'''
    return i_q_set;
}
'''
source += r'''
float integral(const PIDRegulator& regulator) { return regulator.get_integral_error() * regulator.get_config().ki; }
void near(float actual, float expected) { assert(std::fabs(actual-expected)<1e-5f); }
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
    near(separate.servo_torque(),2);
    separate.update_servo_config(SetPointType::POSITION,{.ki=4});
    near(separate.servo_torque(),.0001f); near(separate.servo_torque(),.0002f);
    separate.update_servo_config(SetPointType::POSITION,{.kd=3});
    near(separate.servo_torque(),-.75f);
    separate.set_angle_point(2); near(separate.servo_torque(),-.75f);
    separate.set_velocity_point(1);
    separate.update_servo_config(SetPointType::VELOCITY,{.kp=2});
    near(separate.servo_torque(),1.5f);
    separate.update_servo_config(SetPointType::VELOCITY,{.ki=4});
    near(separate.servo_torque(),.000075f); near(separate.servo_torque(),.00015f);
    FOC motor;
    motor.update_servo_config(SetPointType::POSITION, {.kp=2, .ki=4, .kd=3});
    assert(motor.set_angle_point(2));
    motor.shaft_angle=1; motor.shaft_velocity=.5f;
    near(motor.servo_torque(), .5001f);
    near(integral(motor.servo_pos_reg), .0001f);
    assert(motor.set_angle_point(3));
    near(integral(motor.servo_pos_reg), .0001f); // Same-mode target preserves I.
    near(motor.servo_torque(), 2.5003f); // No D kick from a target step.
    auto gains=motor.get_servo_config(SetPointType::POSITION);
    primask=1; motor.update_servo_config(SetPointType::POSITION,gains);
    assert(primask==1); near(integral(motor.servo_pos_reg),.0003f);
    primask=0; gains.kp=1; motor.update_servo_config(SetPointType::POSITION,gains);
    assert(primask==0); near(integral(motor.servo_pos_reg),0); near(motor.target,3);

    motor.update_servo_config(SetPointType::VELOCITY,{.kp=2,.ki=4});
    assert(motor.set_velocity_point(1));
    near(integral(motor.servo_pos_reg),0);
    near(motor.servo_torque(),1.00005f);
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
    near(motor.servo_torque(),0);

    // Both saturation directions, unwind while saturated, and a reduced current limit.
    motor.update_servo_config(SetPointType::POSITION,{.kp=10,.ki=4});
    motor.drive_runtime_config.user_torque_limit=2;
    motor.shaft_angle=0; motor.shaft_velocity=0;
    for (float sign : {-1.f,1.f}) {
        motor.servo_pos_reg.set_integral_error((0) / motor.servo_pos_reg.get_config().ki); motor.set_angle_point(sign);
        near(motor.servo_torque(),2*sign); near(integral(motor.servo_pos_reg),0);
    }
    motor.servo_pos_reg.set_integral_error((2) / motor.servo_pos_reg.get_config().ki); motor.set_angle_point(-.01f);
    motor.servo_torque(); assert(integral(motor.servo_pos_reg)<2);
    motor.drive_runtime_config.current_limit=.25f;
    motor.servo_torque(); assert(std::fabs(integral(motor.servo_pos_reg))<=.25f);
    motor.drive_runtime_config.current_limit=30;
    motor.update_servo_config(SetPointType::POSITION,{.ki=4,.kd=1});
    motor.servo_pos_reg.set_integral_error((2) / motor.servo_pos_reg.get_config().ki); motor.shaft_velocity=-4;
    near(motor.servo_torque(),2); assert(integral(motor.servo_pos_reg)<2); // Unwind despite saturation.
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
        near(servo_current(motor),.25f*direction);
        motor.drive_info.common.gear_ratio=8;
        near(servo_current(motor),.125f*direction);
        motor.drive_info.common.gear_ratio=4;
    }
    VBDrive device;
    device.update_servo_config(SetPointType::POSITION,{.kp=1,.ki=1});
    device.start(); device.set_angle_point(1); device.servo_torque();
    assert(integral(device.servo_pos_reg)>0);
    device.stop(); assert(!device._is_on && !device.bridge_enabled);
    near(integral(device.servo_pos_reg),0); near(device.target,0);
    device.set_angle_point(2); // A target received while off must not resume on enable.
    device.start(); assert(device._is_on && device.bridge_enabled);
    assert(device.point_type==SetPointType::VOLTAGE); near(device.target,0);
    near(integral(device.servo_pos_reg),0);
    puts("Servo numerical and command-state checks passed");
}
'''
with tempfile.TemporaryDirectory(prefix="vbdrive-servo-") as temporary:
    path = Path(temporary)
    (path / "test.cpp").write_text(source)
    subprocess.run(["c++", "-std=c++20", "-Wall", "-Wextra", "-I"+str(ROOT / "Drivers/libvoltbro"), str(path / "test.cpp"), "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
