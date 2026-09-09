#!/usr/bin/env python3
"""Host regressions for actual catalog, Serial controller and Cyphal register callback.

Only hardware/transport are doubled. Code fragments are extracted unchanged from
the firmware so these tests exercise its implementation, not a Python model.
Requires a C++20 compiler and C/DSDL headers from a configured Release build.
"""
from pathlib import Path
import subprocess
import tempfile
import os

ROOT = Path(__file__).resolve().parents[1]

STUB = r'''
#pragma once
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <climits>
#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <cstdint>
#define arm_atomic(T) T
#define HAL_UART_MODULE_ENABLED
#define HAL_OK 0
using HAL_StatusTypeDef = int;
#define HAL_UART_STATE_BUSY 99
using CanardNodeID = uint8_t;
constexpr unsigned CANARD_NODE_ID_MAX = 127;
struct UART_HandleTypeDef {};
inline std::string uart_output;
inline int resets = 0, boots = 0;
inline int HAL_UART_GetState(UART_HandleTypeDef*) { return 0; }
inline int HAL_UART_Transmit_DMA(UART_HandleTypeDef*,uint8_t* p,size_t n) {
    uart_output.assign(reinterpret_cast<char*>(p), n); return 0;
}
inline int HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,size_t,int) { return 0; }
inline void NVIC_SystemReset() { ++resets; }
inline void Error_Handler() { assert(false); }
#define npf_vsnprintf vsnprintf
struct EEPROM {
    int writes = 0;
    std::array<uint8_t,16384> memory{};
    template<class T> int write(T* value,uint16_t address) {
        if (address==0) ++writes;
        assert(address+sizeof(T)<=memory.size());
        std::memcpy(memory.data()+address,value,sizeof(T)); return 0;
    }
    template<class T> int read(T* value,uint16_t address) {
        assert(address+sizeof(T)<=memory.size());
        std::memcpy(value,memory.data()+address,sizeof(T)); return 0;
    }
};
enum class AngleEncoderType : uint8_t { ROTOR, SHAFT };
struct CalibrationData { char bytes[8]; };
template<class T> constexpr auto to_underlying(T v) { return static_cast<uint8_t>(v); }
template<class T> T value_or_default(T v,T d) { return std::isnan(v)?d:v; }
template<class T> T value_or_default(T v,T d,T unset) { return v==unset?d:v; }
struct DriveRuntimeConfig {
    float user_current_limit=1, user_speed_limit=2, user_torque_limit=3;
    float user_angle_offset=0, user_position_lower_limit=-1, user_position_upper_limit=1;
    int8_t user_angle_direction=1;
};
struct FOCTarget { float torque=0,angle=0,velocity=0,angle_kp=0,velocity_kp=0; };
struct VBInverter {
    float get_mcu_temperature() const {return 30;}
    float get_stator_temperature() const {return 25;}
};
struct VBDrive {
    bool on=true;
    DriveRuntimeConfig limits;
    VBInverter inverter;
    FOCTarget target;
    auto get_runtime_config() const { return limits; }
    bool set_runtime_config(DriveRuntimeConfig v) { if (v.user_current_limit < 0) return false; limits=v; return true; }
    int set_state(bool v) {on=v;return 0;}
    bool is_on() const {return on;}
    auto& get_inverter() const {return inverter;}
    float get_voltage() const {return 24;}
    float get_working_current() const {return .1f;}
    uint32_t get_shaft_encoder_value() const {return 123;}
    uint32_t get_rotor_encoder_value() const {return 456;}
    void set_foc_point(FOCTarget v) {target=v;}
};
'''

TEST = r'''
std::string command(std::string_view s) {
    uart_output.clear(); manager.process_command(s); return uart_output;
}
int main() {
    static_assert(sizeof(VBDriveConfig)==98);
    static_assert(CONFIG_PLACEMENT==0 && CALIBRATION_PLACEMENT==99);
    auto& config = manager.get_config();
    config.node_id=11; config.gear_ratio=36; config.was_configured=true;
    manager.set_state(CommandState::RUNNING);
    assert(command("firmware_rev:?\r\n").find("0123456789abcdef")!=std::string::npos);
    for (size_t i=0; i<PARAMETER_CATALOG.size();++i) {
        const auto& d=PARAMETER_CATALOG[i];
        auto q=command(std::string(d.name)+":?");
        const bool serial = d.id != ParameterId::BOOTLOADER && d.id != ParameterId::CMD_ERRORS;
        assert(serial ? q.find(std::string(d.name)+":")==0 : q.find("Unknown parameter")!=std::string::npos);
        uavcan_register_Value_1_0 in{},out{}; RegisterAccessResponse response{};
        handle_parameter_register(i,in,out,response);
        assert(out._tag_!=REGISTER_EMPTY_TAG);
        assert(response.persistent==d.is_persistent && response._mutable==d.is_mutable);
        ParameterValue value; assert(read_parameter(config,d.id,value));
        switch(d.type) {
            case ParameterType::REAL32: { float v; assert(parse_register_real32(out,v)); float want=std::get<float>(value); assert(v==want || (std::isnan(v)&&std::isnan(want))); break; }
            case ParameterType::NATURAL32: { uint32_t v; assert(parse_register_natural32(out,v)&&v==std::get<uint32_t>(value)); break; }
            case ParameterType::INTEGER32: { int32_t v; assert(parse_register_integer32(out,v)&&v==std::get<int32_t>(value)); break; }
            case ParameterType::BIT: { bool v; assert(parse_register_bit(out,v)&&v==std::get<bool>(value)); break; }
            case ParameterType::STRING: assert(std::string_view(reinterpret_cast<char*>(out._string.value.elements),out._string.value.count)==std::get<std::string_view>(value)); break;
        }
        if (!d.is_mutable) {
            assert(command(std::string(d.name)+":1").find(serial ? "Read-only" : "Unknown parameter")!=std::string::npos);
            fill_register_natural32(in,999); handle_parameter_register(i,in,out,response);
            ParameterValue after; assert(read_parameter(config,d.id,after)); assert(value==after);
        }
    }
    assert(command("kp:5").find("CONFIG mode required")!=std::string::npos);
    command("CONFIG"); command("kp:8"); command("CONFIG");
    assert(config.kp==8); command("kp:bad"); command("EXIT");
    assert(std::isnan(config.kp) && eeprom.writes==0 && manager.is_app_running());
    command("CONFIG"); command("RESET"); command("EXIT"); assert(config.node_id==11);
    for (const auto& d:PARAMETER_CATALOG) {
        if (!d.is_persistent) continue;
        ParameterValue before; assert(read_parameter(config,d.id,before));
        command("CONFIG");
        const auto input=d.type==ParameterType::INTEGER32 ? "-1" : "1";
        assert(command(std::string(d.name)+":"+input).find("OK:")==0);
        command("EXIT");
        ParameterValue after; assert(read_parameter(config,d.id,after));
        if (d.type==ParameterType::REAL32 && std::isnan(std::get<float>(before))) assert(std::isnan(std::get<float>(after)));
        else assert(before==after);
    }
    command("CONFIG"); command("kp:9"); command("kp:bad"); command("SAVE");
    assert(eeprom.writes==1 && config.kp==9);
    command("CONFIG"); command("ang_dir:-1"); assert(config.angle_direction==-1);
    assert(command("ang_dir:0").find("Invalid")!=std::string::npos);
    assert(command("gear:0").find("Invalid")!=std::string::npos);
    assert(command("node_id:128").find("Invalid")!=std::string::npos);
    assert(command("data_baud:4").find("Invalid")!=std::string::npos);
    command("EXIT"); assert(config.angle_direction==1);
    command("TEST"); command("do_vel:0.15"); assert(device.target.velocity==.15f);
    command("ang_off:0.2"); assert(device.limits.user_angle_offset==.2f);
    command("do_free"); assert(device.target.velocity==0); command("STOP");
    assert(manager.is_app_running()); command("is_on:0"); assert(!device.on); command("is_on:1");
    auto access=[&](std::string_view name,uavcan_register_Value_1_0 in) {
        auto* d=find_parameter(name); assert(d);
        uavcan_register_Value_1_0 out{}; RegisterAccessResponse r{};
        handle_parameter_register(d-PARAMETER_CATALOG.data(),in,out,r); return out;
    };
    uavcan_register_Value_1_0 input{};
    fill_register_integer32(input,12); access("node_id",input); assert(config.node_id==12 && config_save_pending);
    int saved=eeprom.writes; persist_pending_config_if_needed(); assert(eeprom.writes==saved+1);
    fill_register_integer32(input,-1); access("ang_dir",input); assert(config.angle_direction==-1 && device.limits.user_angle_direction==-1);
    for (const auto& d:PARAMETER_CATALOG) {
        if (!d.is_persistent) continue;
        if (d.type==ParameterType::REAL32) fill_register_real32(input,1);
        else if (d.type==ParameterType::INTEGER32) fill_register_integer32(input,1);
        else fill_register_natural32(input,1);
        auto out=access(d.name,input);
        assert(out._tag_==input._tag_);
        if(d.type==ParameterType::REAL32) assert(out.real32.value.elements[0]==1);
        else if(d.type==ParameterType::INTEGER32) assert(out.integer32.value.elements[0]==1);
        else assert(out.natural32.value.elements[0]==1);
    }
    fill_register_integer32(input,0); access("is_on",input); assert(!device.on);
    motor=nullptr; fill_register_natural32(input,13); access("node_id",input); assert(config.node_id==13);
    assert(access("encoder_rotor",{})._tag_==REGISTER_EMPTY_TAG); motor=&device;
    for (auto state : {CommandState::RUNNING, CommandState::CONFIG, CommandState::TESTING}) {
        manager.set_state(state);
        for (auto name : {"bootloader", "cmd_errors"}) {
            for (auto value : {"?", "0", "1"}) {
                assert(command(std::string(name)+":"+value).find("Unknown parameter")!=std::string::npos);
            }
        }
        for (const auto& d : PARAMETER_CATALOG) {
            if (d.id != ParameterId::BOOTLOADER && d.id != ParameterId::CMD_ERRORS)
                assert(command(std::string(d.name)+":?").find(std::string(d.name)+":")==0);
        }
        assert(!bootloader_reboot_pending && boots==0);
    }
    manager.set_state(CommandState::RUNNING);
    fill_register_bit(input,true); access("bootloader",input);
    assert(bootloader_reboot_pending); reboot_to_bootloader_if_requested(); assert(boots==1);
    command("BOOT"); assert(boots==2);
    // All six Servo fields persist in the same config write.
    std::fill(eeprom.memory.begin()+sizeof(VBDriveConfig),eeprom.memory.end(),0xA5);
    command("CONFIG");
    command("servo_pos_p_gain:1"); command("servo_pos_i_gain:2");
    command("servo_vel_p_gain:3"); command("servo_vel_i_gain:4");
    command("servo_tr_form:2"); command("servo_tr_vel:5");
    command("SAVE");
    VBDriveConfig loaded;
    assert(eeprom.read(&loaded,0)==HAL_OK);
    assert(loaded.gear_ratio==config.gear_ratio && loaded.servo_pos_p_gain==1);
    assert(loaded.servo_pos_i_gain==2 && loaded.servo_vel_p_gain==3 && loaded.servo_vel_i_gain==4);
    assert(loaded.servo_transient_form==2 && loaded.servo_transient_vel==5);
    command("CONFIG"); command("RESET");
    assert(config.servo_pos_p_gain==0 && config.servo_transient_form==1);
    command("EXIT");
    assert(config.servo_pos_p_gain==1 && config.servo_transient_form==2);
    for (size_t i=sizeof(VBDriveConfig);i<eeprom.memory.size();++i) assert(eeprom.memory[i]==0xA5);
    for (auto name : {"servo_pos_p_gain","servo_pos_i_gain","servo_vel_p_gain","servo_vel_i_gain","servo_tr_vel"}) {
        auto* d=find_parameter(name);
        assert(write_persistent_parameter(config,d->id,-1.0f,false)==ParameterWriteResult::INVALID);
        assert(write_persistent_parameter(config,d->id,NAN,false)==ParameterWriteResult::INVALID);
    }
    assert(write_persistent_parameter(config,ParameterId::SERVO_TR_FORM,uint32_t(3),false)==ParameterWriteResult::INVALID);
    puts("PASS: 37 shared parameters, 2 Cyphal-only registers, Serial/Cyphal writes and rollback, unified config persistence, boot commands");
}
'''

with tempfile.TemporaryDirectory(prefix="vbdrive-interfaces-") as temp:
    tmp = Path(temp)
    for header in ("stm32g4xx_hal.h", "usart.h", "cyphal/cyphal.h",
                   "voltbro/eeprom/eeprom.hpp", "voltbro/motors/bldc/vbdrive/vbdrive.hpp", "nanoprintf.h"):
        dest = tmp / header
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text('#include "stub.hpp"\n')
    (tmp / "stub.hpp").write_text(STUB)
    sm = (ROOT / "App/state_manager.cpp").read_text()
    parsers = sm[sm.index("static constexpr size_t PARSED_VALUE_MAX_SIZE"):sm.index("constexpr auto make_action")]
    config_methods = sm[sm.index("bool VBDriveConfig::are_required_params_set"):]
    app = (ROOT / "App/app.cpp").read_text()
    deferred = app[app.index("static void persist_pending_config_if_needed() {"):app.index("void in_loop_reporting")]
    callback = app[app.index("static void handle_parameter_register"):app.index("void setup_subscriptions() {")]
    utils = (ROOT / "Drivers/libcxxcanard/cyphal/node/registers_utils.hpp").read_text().replace("#include <cyphal/node/registers_handler.hpp>", "")
    parameters = (ROOT / "App/parameters.cpp").read_text().replace('#include "app.h"', '')
    source = '#include "stub.hpp"\n#include "state_manager.hpp"\n#include <uavcan/_register/Access_1_0.h>\n'
    source += 'using RegisterAccessResponse=uavcan_register_Access_Response_1_0;\n'
    source += '''
VBDrive device; VBDrive* motor=&device;
VBDrive* get_motor(){return motor;}
EEPROM eeprom; EEPROM& get_eeprom(){return eeprom;}
UART_HandleTypeDef uart;
DriveStateController manager(&uart,eeprom,[]{device.on=true;},[]{device.on=false;},{});
DriveStateController& get_app_manager(){return manager;}
void reboot_to_bootloader(){++boots;}
bool config_save_pending=false;
#define HAL_IMPORTANT(x) assert((x)==0);
'''
    source += parsers + config_methods + parameters + utils + deferred + callback + TEST
    (tmp / "test.cpp").write_text(source)
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++20", "-DSTM32G4",
                    '-DVBDRIVE_MODEL="M4310"', '-DVBDRIVE_FIRMWARE_REV="0123456789abcdef"',
                    "-I"+str(tmp), "-I"+str(ROOT/"App"), "-I"+str(ROOT/"Drivers/libvoltbro"),
                    "-I"+str(ROOT/"build/Release/cyphal_types/c"), str(tmp/"test.cpp"), "-o", str(tmp/"test")], check=True)
    subprocess.run([str(tmp/"test")], check=True)
