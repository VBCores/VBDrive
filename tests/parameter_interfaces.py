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
#include <vector>
#include <string>
#include <string_view>
#include <cstdint>
#define arm_atomic(T) T
#define HAL_UART_MODULE_ENABLED
#define HAL_OK 0
using HAL_StatusTypeDef = int;
#define HAL_UART_STATE_READY 0
using millis = uint32_t;
inline millis test_millis = 0;
inline millis millis_32() {return test_millis;}
struct FakeSCB {uint32_t ICSR=0;};
inline FakeSCB fake_scb;
#define SCB (&fake_scb)
#define SCB_ICSR_PENDSVSET_Msk (1u<<28)
#define HAL_UART_STATE_BUSY 99
#define HAL_UART_STATE_BUSY_TX 98
#define CRITICAL_SECTION(code) { code }
using CanardNodeID = uint8_t;
constexpr unsigned CANARD_NODE_ID_MAX = 127;
struct UART_HandleTypeDef { int gState=0; };
inline std::string uart_output;
inline std::vector<std::string> uart_frames;
inline int resets = 0, boots = 0, calibrations = 0;
inline int HAL_UART_GetState(UART_HandleTypeDef*) { return 0; }
inline int HAL_UART_Transmit_DMA(UART_HandleTypeDef*,uint8_t* p,size_t n) {
    uart_output.assign(reinterpret_cast<char*>(p), n); uart_frames.push_back(uart_output); return 0;
}
inline int HAL_UART_Transmit(UART_HandleTypeDef* u,uint8_t* p,size_t n,int) { return HAL_UART_Transmit_DMA(u,p,n); }
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
enum class SetPointType { POSITION, VELOCITY };
struct PIDConfig { float kp=0, ki=0, kd=0; };
struct VBInverter {
    float get_mcu_temperature() const {return 30;}
    float get_stator_temperature() const {return 25;}
};
struct VBDrive {
    PIDConfig position, velocity;
    PIDConfig get_servo_config(SetPointType type) const {return type==SetPointType::POSITION?position:velocity;}
    void update_servo_config(SetPointType type,PIDConfig config) {(type==SetPointType::POSITION?position:velocity)=config;}
    bool on=true;
    int stop() {on=false;return 0;}
    float get_angle() const {return 0;}
    float get_velocity() const {return 0;}
    float get_torque() const {return 0;}
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
    int servo_type=-1;
    float servo_value=0;
    bool set_velocity_point(float v) {if (!valid_target) return false; servo_type=0; servo_value=v; return true;}
    bool set_torque_point(float v) {if (!valid_target) return false; servo_type=1; servo_value=v; return true;}
    bool set_angle_point(float v) {if (!valid_target) return false; servo_type=2; servo_value=v; return true;}
    bool set_voltage_point(float v) {if (!valid_target) return false; servo_type=3; servo_value=v; return true;}
    bool valid_target=true;
    bool set_foc_point(FOCTarget v) {if (!valid_target) return false; target=v; return true;}
};
'''

TEST = r'''
std::string command(std::string_view s) {
    uart_output.clear(); manager.process_command(s); return uart_output;
}
int main() {
    {
        char buffer[16];
        UART_HandleTypeDef uart;
        { UARTResponseAccumulator response(&uart,buffer,sizeof(buffer));
          response.append("%s","012345678901234567890123456789"); }
        assert(uart_output.size()<=sizeof(buffer));
        uart_output.clear();
        uart_frames.clear();
        { UARTResponseAccumulator response(&uart,buffer,sizeof(buffer),true);
          for (int i=0;i<100;++i) response.append("line\r\n"); }
        assert(uart_frames.size()==100);
        for (const auto& frame : uart_frames) assert(frame=="line\r\n");
    }
    static_assert(sizeof(VBDriveConfig)==102);
    static_assert(CONFIG_PLACEMENT==0 && CALIBRATION_PLACEMENT==103);
    auto& config = manager.get_config();
    config.apply_servo_config();
    assert(device.position.kp==150 && device.position.ki==200 && device.position.kd==10);
    assert(device.velocity.kp==30 && device.velocity.ki==60);
    config.node_id=11; config.gear_ratio=36; config.was_configured=true;
    {
        char buffer[512];
        UART_HandleTypeDef uart;
        uart_output.clear();
        uart_frames.clear();
        { UARTResponseAccumulator response(&uart,buffer,sizeof(buffer),true);
          config.print_self(response); }
        std::string dump;
        for (const auto& frame : uart_frames) dump+=frame;
        assert(dump.size()>512);
        assert(dump.find("servo_tr_vel:")!=std::string::npos);
        assert(dump.find("are all required params set: true")!=std::string::npos);
    }
    manager.set_state(CommandState::RUNNING);
    assert(command("firmware_rev:?\r\n").find("0123456789abcdef")!=std::string::npos);
    for (size_t i=0; i<PARAMETER_CATALOG.size();++i) {
        const auto& d=PARAMETER_CATALOG[i];
        auto q=command(std::string(d.name)+":?");
        const bool serial = true;
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
    command("mit_cmd: 0 0.15 0 0 0.5"); assert(device.target.velocity==.15f);
    assert(command("ang_off:0.2").find("CONFIG mode required")!=std::string::npos);
    command("STOP"); assert(device.servo_type==3 && device.servo_value==0);
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
    assert(command("STOP").find("OK: STOP")==0);
    assert(access("encoder_rotor",{})._tag_==REGISTER_EMPTY_TAG); motor=&device;
    for (auto state : {CommandState::INIT, CommandState::RUNNING, CommandState::CONFIG, CommandState::NOT_CALIBRATED}) {
        manager.set_state(state);
        for (const auto& d : PARAMETER_CATALOG) {
            assert(command(std::string(d.name)+":?").find(std::string(d.name)+":")==0);
            if (!d.is_mutable) assert(command(std::string(d.name)+":1").find("Read-only")!=std::string::npos);
        }
        assert(command("bootloader:0").find("OK:")==0);
        assert(!bootloader_reboot_pending);
        assert(command("STOP").find("OK: STOP")==0);
        assert(device.servo_type==3 && device.servo_value==0);
        assert(manager.get_state()==state);
    }
    manager.set_state(CommandState::RUNNING);
    fill_register_bit(input,true); access("bootloader",input);
    assert(bootloader_reboot_pending); reboot_to_bootloader_if_requested(); assert(boots==1);
    command("bootloader:1"); command("bootloader:0");
    assert(bootloader_reboot_pending); reboot_to_bootloader_if_requested(); assert(boots==2);
    assert(command("BOOT").find("Unknown command")!=std::string::npos);
    for (auto old : {"TEST","do_vel:1","do_ang:1","do_free"}) assert(command(old).find("Unknown")!=std::string::npos);
    // All seven Servo fields persist in the same config write.
    std::fill(eeprom.memory.begin()+sizeof(VBDriveConfig),eeprom.memory.end(),0xA5);
    command("CONFIG");
    command("servo_pos_p_gain:1"); command("servo_pos_i_gain:2");
    command("servo_pos_d_gain:0.5");
    command("servo_vel_p_gain:3"); command("servo_vel_i_gain:4");
    command("servo_tr_form:2"); command("servo_tr_vel:5");
    command("SAVE");
    VBDriveConfig loaded;
    assert(eeprom.read(&loaded,0)==HAL_OK);
    assert(loaded.gear_ratio==config.gear_ratio && loaded.servo_pos_p_gain==1);
    assert(loaded.servo_pos_i_gain==2 && loaded.servo_vel_p_gain==3 && loaded.servo_vel_i_gain==4);
    assert(loaded.servo_pos_d_gain==.5f && device.position.kd==.5f);
    assert(loaded.servo_transient_form==2 && loaded.servo_transient_vel==5);
    command("CONFIG"); command("RESET");
    assert(std::isnan(config.servo_pos_p_gain) && std::isnan(config.servo_pos_i_gain) && std::isnan(config.servo_pos_d_gain));
    assert(std::isnan(config.servo_vel_p_gain) && std::isnan(config.servo_vel_i_gain) && std::isnan(config.servo_transient_vel));
    assert(config.servo_transient_form==0);
    assert(command("servo_tr_form:?").find("servo_tr_form:1")!=std::string::npos);
    assert(command("servo_tr_vel:?").find("0.000000")!=std::string::npos);
    assert(command("servo_pos_p_gain:?").find("150.000000")!=std::string::npos);
    assert(command("servo_pos_i_gain:?").find("200.000000")!=std::string::npos);
    assert(command("servo_pos_d_gain:?").find("10.000000")!=std::string::npos);
    assert(command("servo_vel_p_gain:?").find("30.000000")!=std::string::npos);
    assert(command("servo_vel_i_gain:?").find("60.000000")!=std::string::npos);
    command("EXIT");
    assert(config.servo_pos_p_gain==1 && config.servo_transient_form==2);
    for (size_t i=sizeof(VBDriveConfig);i<eeprom.memory.size();++i) assert(eeprom.memory[i]==0xA5);
    for (auto name : {"servo_pos_p_gain","servo_pos_i_gain","servo_pos_d_gain","servo_vel_p_gain","servo_vel_i_gain","servo_tr_vel"}) {
        auto* d=find_parameter(name);
        assert(write_persistent_parameter(config,d->id,-1.0f,false)==ParameterWriteResult::INVALID);
        assert(write_persistent_parameter(config,d->id,NAN,false)==ParameterWriteResult::INVALID);
    }
    assert(write_persistent_parameter(config,ParameterId::SERVO_TR_FORM,uint32_t(3),false)==ParameterWriteResult::INVALID);
    command("CONFIG"); command("servo_pos_p_gain:9");
    assert(device.position.kp==1);
    fill_register_real32(input,7); access("servo_pos_i_gain",input);
    assert(device.position.kp==1 && device.position.ki==7);
    persist_pending_config_if_needed(); eeprom.read(&loaded,0);
    assert(loaded.servo_pos_p_gain==1 && loaded.servo_pos_i_gain==7);
    command("EXIT");
    assert(config.servo_pos_p_gain==1 && config.servo_pos_i_gain==7);
    command("CONFIG"); command("servo_pos_d_gain:0.75"); command("APPLY");
    eeprom.read(&loaded,0); assert(loaded.servo_pos_d_gain==.75f);
    assert(device.position.kd==.5f); // APPLY reloads on the actual reset, not before it.
    loaded.apply_servo_config(); assert(device.position.kd==.75f);
    manager.set_state(CommandState::RUNNING); command("TEST"); command("do_vel:0.15");
    unsigned errors=access("cmd_errors",{}).natural32.value.elements[0];
    for (auto invalid : {"mit_cmd: 0 nan 0 0 0", "servo_cmd: 2 inf", "servo_cmd: 0 -inf", "servo_cmd: 0 bad"}) {
        assert(command(invalid).find("Invalid target")!=std::string::npos);
        assert(device.target.velocity==.15f);
    }
    device.valid_target=false;
    assert(command("servo_cmd: 0 5").find("Invalid target")!=std::string::npos);
    assert(device.target.velocity==.15f);
    assert(access("cmd_errors",{}).natural32.value.elements[0]==errors+5);
    device.valid_target=true; command("STOP");
    // Exact motion grammar, common dispatch, and transport-independent errors.
    for (int type=0; type<4; ++type) {
        assert(command("servo_cmd: "+std::to_string(type)+" -0.25").find("OK:")==0);
        assert(device.servo_type==type && device.servo_value==-.25f);
    }
    assert(command("mit_cmd:\t1 2 3 4 5").find("OK:")==0);
    assert(device.target.angle==1 && device.target.velocity==2 && device.target.torque==3);
    assert(device.target.angle_kp==4 && device.target.velocity_kp==5);
    for (auto bad : {"mit_cmd: 1 2 3 4", "mit_cmd: 1 2 3 4 5 6", "mit_cmd: 1 2 3 bad 5",
                     "servo_cmd: 1.0 2", "servo_cmd: 4 2", "servo_cmd: -1 2", "servo_cmd: 0 2x", "servo_cmd: 0"}) {
        auto errors_before=access("cmd_errors",{}).natural32.value.elements[0];
        assert(command(bad).find("Invalid target")!=std::string::npos);
        assert(access("cmd_errors",{}).natural32.value.elements[0]==errors_before+1);
        assert(device.target.angle==1 && device.servo_value==-.25f);
    }
    command("log_on"); assert(manager.is_logging());
    command("STOP"); assert(manager.is_logging());
    command("CONFIG"); assert(!manager.is_logging());
    assert(command("mit_cmd: 0 0 0 0 0").find("RUNNING mode required")!=std::string::npos);
    command("EXIT"); assert(!manager.is_logging());
    command("log_on"); manager.set_state(CommandState::CALIBRATING); assert(!manager.is_logging());
    discard_serial_input();
    uart_frames.clear();
    receive_serial("STOP\nbootloader:1\nservo_cmd: 0 1\n");
    assert(serial_head == serial_tail && uart_frames.empty());
    manager.set_state(CommandState::RUNNING);
    receive_serial("servo_cmd: 0 "); drain_serial();
    receive_serial("1\nSTOP\n");
    manager.set_state(CommandState::CALIBRATING);
    discard_serial_input();
    receive_serial("servo_cmd: 0 ");
    manager.set_state(CommandState::RUNNING);
    receive_serial("1\nfirmware_rev:?\n"); drain_serial();
    assert(uart_frames.size()==1 && uart_frames[0].find("firmware_rev:")==0);

    // Real FIFO and line consumer: fragments, CR/LF/CRLF, bursts, recovery.
    uart_frames.clear();
    receive_serial("servo_cmd: 0 "); drain_serial(); assert(uart_frames.empty());
    receive_serial("0.125\r\nSTOP\nfirmware_rev:?\r"); drain_serial();
    assert(uart_frames.size()==3 && uart_frames[0].find("OK: servo_cmd")==0);
    assert(uart_frames[1].find("OK: STOP")==0 && uart_frames[2].find("firmware_rev:")==0);
    uart_frames.clear();
    const std::string long_command="mit_cmd: 0.000000000000000000 0.000000000000000000 0 0 0\n";
    for (char byte : long_command) {receive_serial(std::string_view(&byte,1)); drain_serial();}
    assert(uart_frames.size()==1 && uart_frames[0].find("OK: mit_cmd")==0);
    uart_frames.clear();
    receive_serial(std::string(220,'x')+"\nSTOP\n"); drain_serial();
    assert(uart_frames.size()==2 && uart_frames[0].find("ERROR:")==0 && uart_frames[1].find("OK: STOP")==0);
    uart_frames.clear();
    receive_serial(std::string(600,'x')+"\nSTOP\n"); drain_serial();
    assert(uart_frames.size()==2 && uart_frames[0].find("overflow")!=std::string::npos && uart_frames[1].find("OK: STOP")==0);
    // One bounded command per service, no consumption while TX owns the buffer.
    uart_frames.clear();
    receive_serial("STOP\nfirmware_rev:?\n");
    auto tail_before = serial_tail;
    uart.gState = HAL_UART_STATE_BUSY_TX;
    serial_service();
    assert(serial_tail==tail_before && uart_frames.empty());
    uart.gState = HAL_UART_STATE_READY;
    serial_service();
    assert(uart_frames.size()==1 && serial_head!=serial_tail);
    serial_service();
    assert(uart_frames.size()==2 && serial_head==serial_tail);
    // SAVE and APPLY never perform EEPROM/reset from the serial interrupt.
    command("CONFIG"); command("gear:36");
    int writes_before=eeprom.writes;
    receive_serial("SAVE\n"); serial_service();
    assert(serial_deferred && eeprom.writes==writes_before);
    serial_service(); assert(eeprom.writes==writes_before);
    process_serial(); assert(!serial_deferred && eeprom.writes==writes_before+1);
    int resets_before=resets;
    receive_serial("APPLY\n"); serial_service();
    assert(serial_deferred && resets==resets_before);
    process_serial(); assert(!serial_deferred && resets==resets_before+1 && !device.on);
    receive_serial("CALIBRATE\n"); serial_service();
    assert(serial_deferred && calibrations==0);
    process_serial(); assert(!serial_deferred && calibrations==1);
    receive_serial("is_on:1\n"); serial_service();
    assert(serial_deferred && !device.on);
    process_serial(); assert(!serial_deferred && device.on);
    for (auto state : {CommandState::RUNNING, CommandState::CONFIG}) {
        manager.set_state(state);
        const auto writes_before_info=eeprom.writes;
        for (auto name : {"INFO", "HELP"}) {
            uart_frames.clear();
            receive_serial(std::string(name)+"\r\n"); serial_service();
            assert(serial_deferred && uart_frames.empty());
            process_serial();
            std::string output;
            for (const auto& frame : uart_frames) output+=frame;
            assert(output.size()>512);
            if (std::string_view(name)=="INFO") {
                assert(output.starts_with("Got config_data type_id:"));
                assert(output.find("servo_tr_vel:")!=std::string::npos);
                assert(output.ends_with("See HELP for available commands\r\n"));
            } else {
                for (auto token : {"INFO", "HELP", "CONFIG", "EXIT", "SAVE", "RESET", "APPLY",
                                   "CALIBRATE", "STOP", "mit_cmd:", "servo_cmd:", "log_on", "log_off",
                                   "<parameter>:?", "is_on:0/1", "bootloader:1"})
                    assert(output.find(token)!=std::string::npos);
            }
            assert(manager.get_state()==state && device.on && eeprom.writes==writes_before_info);
            serial_service(); // Consume trailing LF.
        }
    }
    // INFO must reproduce the complete startup output without loading/saving EEPROM.
    eeprom.write(&config, CONFIG_PLACEMENT);
    const int writes_before_startup=eeprom.writes;
    uart_frames.clear(); manager.init();
    std::string startup;
    for (const auto& frame : uart_frames) startup+=frame;
    assert(startup.ends_with("See HELP for available commands\r\n"));
    uart_frames.clear(); command("INFO");
    std::string info;
    for (const auto& frame : uart_frames) info+=frame;
    assert(info==startup && eeprom.writes==writes_before_startup);
    // The 1 kHz scheduler does nothing while foreground work/calibration owns Serial.
    SCB->ICSR=0; serial_busy=true; serial_tick(); assert(SCB->ICSR==0);
    serial_busy=false; serial_tick(); assert(SCB->ICSR==SCB_ICSR_PENDSVSET_Msk);
    // Explicit zero is a value, not the NAN sentinel for a default gain.
    VBDriveConfig zero;
    zero.servo_pos_p_gain=zero.servo_pos_i_gain=zero.servo_pos_d_gain=0;
    zero.servo_vel_p_gain=zero.servo_vel_i_gain=0;
    zero.apply_servo_config();
    assert(device.position.kp==0 && device.position.ki==0 && device.position.kd==0);
    assert(device.velocity.kp==0 && device.velocity.ki==0);
    ParameterValue zero_value;
    assert(read_parameter(zero,ParameterId::SERVO_POS_P_GAIN,zero_value) && std::get<float>(zero_value)==0);
    puts("PASS: 40 shared registers, Serial/Cyphal isolation, Servo apply and persistence, boot commands");
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
DriveStateController manager(&uart,eeprom,[]{device.on=true;},[]{device.on=false;},
    {{"CALIBRATE", {[]{return true;}, []{++calibrations; return true;}}}});
UART_HandleTypeDef& huart2=uart;
bool is_able_to_calibrate() {return true;}
DriveStateController& get_app_manager(){return manager;}
void reboot_to_bootloader(){++boots;}
bool config_save_pending=false;
#define HAL_IMPORTANT(x) assert((x)==0);
'''
    shared = app[app.index("bool apply_mit_command("):app.index("class FOCCommandSub:")]
    receive = sm[sm.index("// DMA producer, bounded PendSV consumer;"):sm.index("void start_uart_recv_it()")]
    source += '#include <voltbro/foc/Servo_1_0.h>\n'
    source += parsers + config_methods + parameters + utils + deferred + callback + shared + receive + '\nvoid drain_serial(){process_serial(); for(int i=0;i<16;++i) serial_service();}\n' + TEST
    (tmp / "test.cpp").write_text(source)
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++20", "-DSTM32G4",
                    '-DVBDRIVE_MODEL="M4310"', '-DVBDRIVE_FIRMWARE_REV="0123456789abcdef"',
                    "-I"+str(tmp), "-I"+str(ROOT/"App"), "-I"+str(ROOT/"Drivers/libvoltbro"),
                    "-I"+str(ROOT/"build/Release/cyphal_types/c"), str(tmp/"test.cpp"), "-o", str(tmp/"test")], check=True)
    subprocess.run([str(tmp/"test")], check=True)
