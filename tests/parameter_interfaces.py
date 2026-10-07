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
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/foc/servo_control.hpp>
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
struct UART_HandleTypeDef { int gState=0; struct {uint32_t BaudRate=115200;} Init; void* hdmarx=nullptr; };
extern UART_HandleTypeDef huart2;
inline int HAL_UART_Init(UART_HandleTypeDef*) {return 0;}
inline int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t*,size_t) {return 0;}
#define DMA_IT_HT 0
#define __HAL_DMA_DISABLE_IT(...) ((void)0)
#define HAL_IMPORTANT(x) assert((x)==0);
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
class EEPROM {
public:
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
struct DriveRuntimeConfig {
    float user_current_limit=1, user_speed_limit=2, user_torque_limit=3;
    float user_angle_offset=0, user_position_lower_limit=-1, user_position_upper_limit=1;
    int8_t user_angle_direction=1;
};
struct FOCTarget { float torque=0,angle=0,velocity=0,angle_kp=0,velocity_kp=0; };
enum class SetPointType { POSITION, VELOCITY, TORQUE, VOLTAGE };
struct PIDConfig { float kp=0, ki=0, kd=0; };
struct VBInverter {
    float get_mcu_temperature() const {return 30;}
    float get_stator_temperature() const {return 25;}
};
struct VBDrive {
    PIDConfig position, velocity;
    PIDConfig get_servo_config(SetPointType type) const {return type==SetPointType::POSITION?position:velocity;}
    void update_servo_config(SetPointType type,PIDConfig config) {(type==SetPointType::POSITION?position:velocity)=config;}
    ServoInputConfig servo_input_config;
    std::optional<ServoCommand> servo_command;
    std::optional<ServoTrajectoryStorage> servo_traj_generator;
    float T=.000025f;
    uint32_t control_tick=0;
    uint8_t servo_reference_ticks=0, servo_integral_ticks=0;
    bool servo_reference_initialize=false;
    float servo_target=0;
    SetPointType point_type=SetPointType::VOLTAGE;
    bool is_velocity_target_valid(float) const {return valid_target;}
    bool is_angle_target_valid(float) const {return valid_target;}
    bool is_torque_target_valid(float) const {return valid_target;}
    float get_direction_multiplier() const {return 1;}
    void reset_control() {servo_reference_ticks=servo_integral_ticks=0;servo_reference_initialize=false;}
    // Production command/configuration methods are injected here below.
    __PRODUCTION_SERVO_METHODS__
    bool on=true;
    unsigned starts=0;int start_result=0;
    int stop() {on=false;return 0;}
    int start() {++starts;on=start_result==0;return start_result;}
    float get_angle() const {return 0;}
    float get_velocity() const {return 0;}
    float get_torque() const {return 0;}
    DriveRuntimeConfig limits;
    VBInverter inverter;
    FOCTarget target;
    auto get_runtime_config() const { return limits; }
    bool set_runtime_config(DriveRuntimeConfig v) { if (v.user_current_limit < 0) return false; limits=v; return true; }
    int set_state(bool v) {if (on==v)return 0;return v ? start() : stop();}
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
    bool set_foc_point(FOCTarget v) {if (!valid_target) return false; reset_servo_input(); target=v; return true;}
    bool set_servo_command(uint8_t type,float value,bool indexed,uint8_t index) {
        if (!valid_target || type>6) return false;
        if (!set_servo_command_impl(type,value,indexed,index)) return false;
        servo_type=type; servo_value=value; return true;
    }
};
'''


def extract_function(text, signature):
    start=text.index(signature); opening=text.index('{',start); depth=1; end=opening+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]

foc_header=(ROOT/'Drivers/libvoltbro/voltbro/motors/bldc/foc/foc.hpp').read_text()
methods='\n'.join(extract_function(foc_header, sig) for sig in
                  ('TrajectoryState servo_initial_state(', 'void reset_servo_input()', 'bool set_servo_input_config(', 'bool set_servo_command('))
methods=methods.replace('bool set_servo_command(', 'bool set_servo_command_impl(').replace('target =', 'servo_target =')
STUB=STUB.replace('__PRODUCTION_SERVO_METHODS__', methods).replace('#include <voltbro/utils.hpp>', '#include <voltbro/utils.hpp>\n#include <voltbro/profiling.hpp>')

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
    static_assert(sizeof(BaseConfigData)==28 && sizeof(VBDriveConfig)==122 && sizeof(DriveConfig)==150);
    static_assert(CONFIG_PLACEMENT==0 && CALIBRATION_PLACEMENT==0x100);
    auto& config = manager.get_config();
    for (const auto& definition : PARAMETER_CATALOG) {
        assert(definition.default_value.has_value() == definition.is_persistent);
        if (!definition.default_value) continue;
        ParameterValue actual;
        assert(read_parameter(config, definition.id, actual));
        if (definition.type == ParameterType::REAL32 && std::isnan(std::get<float>(*definition.default_value))) {
            assert(std::isnan(std::get<float>(actual)));
        } else assert(actual == *definition.default_value);
    }
    config.app.apply_servo_config();
    assert(device.position.kp==150 && device.position.ki==200 && device.position.kd==10);
    assert(device.velocity.kp==30 && device.velocity.ki==60);
    config.base.node_id=11; config.app.gear_ratio=36; config.base.was_configured=true;
    {
        char buffer[512];
        UART_HandleTypeDef uart;
        uart_output.clear();
        uart_frames.clear();
        { UARTResponseAccumulator response(&uart,buffer,sizeof(buffer),true);
          serial_print_config(config,response); }
        std::string dump;
        for (const auto& frame : uart_frames) dump+=frame;
        assert(dump.size()>512);
        assert(dump.find("servo_control_vel_limit:")!=std::string::npos);
        assert(dump.find("are all required params set: true")!=std::string::npos);
    }
    manager.set_state(CommandState::RUNNING);
    assert(command("firmware_rev:?\r\n").find("4.0.0")!=std::string::npos);
    assert(command("name:?").find("name:vbdrive")==0);
    assert(command("device:?")=="device:vbdrive\n\r");
    assert(command("rated_max_torque:?")=="rated_max_torque:30.000000\n\r");
    assert(command("rated_max_current:?")=="rated_max_current:30.000000\n\r");
    assert(command("rated_max_current:17").find("CONFIG mode required")!=std::string::npos);
    command("CONFIG");
    assert(command("rated_max_current:17").find("OK")!=std::string::npos);
    assert(command("rated_max_torque:23").find("OK")!=std::string::npos);
    assert(command("rated_max_current:?")=="rated_max_current:17.000000\n\r");
    command("EXIT");
    assert(std::isnan(config.app.rated_max_current) && std::isnan(config.app.rated_max_torque));
    uart_frames.clear();
    assert(command("CALIBRATE")=="CALIBRATE FINISH\r\n");
    assert(uart_frames.size()==2 && uart_frames[0]=="CALIBRATE OK\r\n" &&
           uart_frames[1]=="CALIBRATE FINISH\r\n");
    assert(calibrations==1);
    manager.set_state(CommandState::CONFIG);
    assert(command("CALIBRATE")=="CALIBRATE ERROR: conditions not met\r\n");
    manager.set_state(CommandState::RUNNING);
    assert(command("log_on")=="log_on OK\r\n");
    assert(command("log_off")=="log_off OK\r\n");
    assert(command("vbdrive_model:?").find("Unknown parameter")!=std::string::npos);
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
    assert(config.app.kp==8); command("kp:bad"); command("EXIT");
    assert(std::isnan(config.app.kp) && eeprom.writes==0 && manager.is_app_running());
    assert(command("CONFIG")=="CONFIG OK: mode enabled\r\n");
    assert(command("RESET").starts_with("RESET OK:"));
    assert(command("EXIT").starts_with("EXIT OK:")); assert(config.base.node_id==11);
    for (const auto& d:PARAMETER_CATALOG) {
        if (!d.is_persistent) continue;
        ParameterValue before; assert(read_parameter(config,d.id,before));
        command("CONFIG");
        const auto input=d.id==ParameterId::SERIAL_BAUD ? "9600" : d.type==ParameterType::INTEGER32 ? "-1" : "1";
        const auto reply=command(std::string(d.name)+":"+input);
        assert(reply.starts_with(std::string(d.name)+":") && reply.ends_with(" OK\r\n"));
        command("EXIT");
        ParameterValue after; assert(read_parameter(config,d.id,after));
        if (d.type==ParameterType::REAL32 && std::isnan(std::get<float>(before))) assert(std::isnan(std::get<float>(after)));
        else assert(before==after);
    }
    command("CONFIG"); command("kp:9"); command("kp:bad"); command("SAVE");
    assert(eeprom.writes==1 && config.app.kp==9);
    assert(command("name:axis-01").find("CONFIG mode required")!=std::string::npos);
    command("CONFIG");
    assert(command("name:left drive").starts_with("name:left drive OK"));
    assert(command("name:?").find("name:left drive")==0);
    assert(command("name:").find("Invalid value")!=std::string::npos);
    assert(command("name:0123456789abcdef").find("Invalid value")!=std::string::npos);
    assert(command("name:123456789012345").starts_with("name:123456789012345 OK"));
    command("EXIT"); assert(command("name:?").find("name:vbdrive")==0);
    command("CONFIG"); command("ang_dir:-1"); assert(config.app.angle_direction==-1);
    assert(command("ang_dir:0").find("Invalid")!=std::string::npos);
    assert(command("gear:0").find("Invalid")!=std::string::npos);
    assert(command("node_id:128").find("Invalid")!=std::string::npos);
    assert(command("data_baud:4").find("Invalid")!=std::string::npos);
    assert(command("serial_baud:0").find("Invalid")!=std::string::npos);
    assert(command("serial_baud:123456").find("Invalid")!=std::string::npos);
    assert(command("serial_baud:230400").find(" OK")!=std::string::npos);
    assert(huart2.Init.BaudRate==115200); // A staged baud change never switches the live UART.
    command("EXIT"); assert(config.app.angle_direction==1);
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
    const int before_node_write=eeprom.writes; fill_register_integer32(input,12); access("node_id",input); assert(config.base.node_id==12 && eeprom.writes==before_node_write);
    int saved=eeprom.writes; manager.persist_pending_config(); assert(eeprom.writes==saved+1);
    fill_register_integer32(input,-1); access("ang_dir",input); assert(config.app.angle_direction==-1 && device.limits.user_angle_direction==1);
    fill_register_integer32(input,1); access("ang_dir",input); assert(config.app.angle_direction==1 && device.limits.user_angle_direction==-1);
    for (const auto& d:PARAMETER_CATALOG) {
        if (!d.is_persistent) continue;
        if (d.type==ParameterType::REAL32) fill_register_real32(input,1);
        else if (d.type==ParameterType::INTEGER32) fill_register_integer32(input,1);
        else if (d.type==ParameterType::STRING) fill_register_string(input,"axis-01");
        else fill_register_natural32(input,d.id==ParameterId::SERIAL_BAUD ? 9600 : 1);
        auto out=access(d.name,input);
        assert(out._tag_==input._tag_);
        if(d.type==ParameterType::REAL32) assert(out.real32.value.elements[0]==1);
        else if(d.type==ParameterType::INTEGER32) assert(out.integer32.value.elements[0]==1);
        else if(d.type==ParameterType::STRING) assert(std::string_view(reinterpret_cast<char*>(out._string.value.elements),out._string.value.count)=="axis-01");
        else assert(out.natural32.value.elements[0]==(d.id==ParameterId::SERIAL_BAUD ? 9600 : 1));
    }
    manager.persist_pending_config();
    DriveConfig named;
    eeprom.read(&named,0); assert(std::string_view(named.base.name)=="axis-01");
    assert(named.app.rated_max_current==1 && named.app.rated_max_torque==1);
    for (auto name : {"rated_max_current", "rated_max_torque"}) {
        for (float bad : {-1.0f, 0.0f, INFINITY, -INFINITY}) {
            fill_register_real32(input,bad);
            assert(access(name,input).real32.value.elements[0]==1);
        }
        fill_register_real32(input,.5f); // Explicit user limit is currently 1.
        assert(access(name,input).real32.value.elements[0]==.5f);
        assert(config.app.max_current==1 && config.app.max_torque==1);
        fill_register_real32(input,NAN);
        assert(access(name,input).real32.value.elements[0]==30);
    }
    for (auto name : {"max_i", "max_tq"}) {
        fill_register_real32(input,31);
        assert(access(name,input).real32.value.elements[0]==31);
    }
    manager.persist_pending_config();
    eeprom.read(&named,0);
    assert(std::isnan(named.app.rated_max_current) && std::isnan(named.app.rated_max_torque));
    assert(named.app.max_current==31 && named.app.max_torque==31);
    assert(command("device:?")=="device:vbdrive\n\r");
    fill_register_string(input,"1234567890123456"); access("name",input);
    assert(std::string_view(config.base.name)=="axis-01");
    fill_register_natural32(input,1); access("name",input);
    assert(std::string_view(config.base.name)=="axis-01");
    assert(write_persistent_parameter(config,ParameterId::NAME,std::string_view("bad\nname"),false)==ParameterWriteResult::INVALID);
    fill_register_integer32(input,0); access("is_on",input); assert(!device.on);
    motor=nullptr; fill_register_natural32(input,13); access("node_id",input); assert(config.base.node_id==13);
    assert(command("STOP")=="STOP OK\r\n");
    assert(access("encoder_rotor",{})._tag_==REGISTER_EMPTY_TAG);
    assert(access("rated_max_torque",{})._tag_==REGISTER_REAL32_TAG);
    assert(access("rated_max_current",{})._tag_==REGISTER_REAL32_TAG);
    auto identity=access("device",{});
    assert(std::string_view(reinterpret_cast<char*>(identity._string.value.elements),identity._string.value.count)=="vbdrive");
    motor=&device;
    for (auto state : {CommandState::INIT, CommandState::RUNNING, CommandState::CONFIG, CommandState::NOT_CALIBRATED}) {
        manager.set_state(state);
        for (const auto& d : PARAMETER_CATALOG) {
            assert(command(std::string(d.name)+":?").find(std::string(d.name)+":")==0);
            if (!d.is_mutable) assert(command(std::string(d.name)+":1").find("Read-only")!=std::string::npos);
        }
        assert(command("bootloader:0")=="bootloader:0 OK\r\n");
        assert(!bootloader_reboot_pending);
        assert(command("STOP")=="STOP OK\r\n");
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
    // All Servo fields persist in the same config write.
    std::fill(eeprom.memory.begin()+sizeof(VBDriveConfig),eeprom.memory.end(),0xA5);
    command("CONFIG");
    command("servo_pos_p_gain:1"); command("servo_pos_i_gain:2");
    command("servo_pos_d_gain:0.5");
    command("servo_vel_p_gain:3"); command("servo_vel_i_gain:4");
    command("servo_control_vel_limit:5"); command("servo_control_accel_limit:2");
    command("rated_max_torque:23"); command("rated_max_current:17");
    command("SAVE");
    DriveConfig loaded;
    assert(eeprom.read(&loaded,0)==HAL_OK);
    assert(loaded.app.rated_max_torque==23 && loaded.app.rated_max_current==17);
    assert(loaded.app.gear_ratio==config.app.gear_ratio && loaded.app.servo_pos_p_gain==1);
    assert(loaded.app.servo_pos_i_gain==2 && loaded.app.servo_vel_p_gain==3 && loaded.app.servo_vel_i_gain==4);
    assert(loaded.app.servo_pos_d_gain==.5f && device.position.kd==.5f);
    assert(loaded.app.servo_control_vel_limit==5 && loaded.app.servo_control_accel_limit==2);
    command("CONFIG"); command("RESET");
    assert(std::isnan(config.app.servo_pos_p_gain) && std::isnan(config.app.servo_pos_i_gain) && std::isnan(config.app.servo_pos_d_gain));
    assert(std::isnan(config.app.servo_vel_p_gain) && std::isnan(config.app.servo_vel_i_gain) && std::isnan(config.app.servo_control_vel_limit));
    assert(command("servo_control_vel_limit:?").find("0.000000")!=std::string::npos);
    assert(command("servo_pos_p_gain:?").find("150.000000")!=std::string::npos);
    assert(command("servo_pos_i_gain:?").find("200.000000")!=std::string::npos);
    assert(command("servo_pos_d_gain:?").find("10.000000")!=std::string::npos);
    assert(command("servo_vel_p_gain:?").find("30.000000")!=std::string::npos);
    assert(command("servo_vel_i_gain:?").find("60.000000")!=std::string::npos);
    command("EXIT");
    assert(config.app.servo_pos_p_gain==1 && config.app.servo_control_vel_limit==5);
    for (size_t i=sizeof(DriveConfig);i<eeprom.memory.size();++i) assert(eeprom.memory[i]==0xA5);
    for (auto name : {"servo_pos_p_gain","servo_pos_i_gain","servo_pos_d_gain","servo_vel_p_gain","servo_vel_i_gain"}) {
        auto* d=find_parameter(name);
        assert(write_persistent_parameter(config,d->id,-1.0f,false)==ParameterWriteResult::INVALID);
        assert(write_persistent_parameter(config,d->id,NAN,false)==ParameterWriteResult::INVALID);
    }
    assert(write_persistent_parameter(config,ParameterId::SERVO_CONTROL_VEL_LIMIT,-1.0f,false)==ParameterWriteResult::INVALID);
    assert(write_persistent_parameter(config,ParameterId::SERVO_CONTROL_VEL_LIMIT,NAN,false)==ParameterWriteResult::OK);
    command("CONFIG"); command("servo_pos_p_gain:9");
    assert(device.position.kp==1);
    fill_register_real32(input,7); access("servo_pos_i_gain",input);
    assert(device.position.kp==1 && device.position.ki==7);
    manager.persist_pending_config(); eeprom.read(&loaded,0);
    assert(loaded.app.servo_pos_p_gain==1 && loaded.app.servo_pos_i_gain==7);
    command("EXIT");
    assert(config.app.servo_pos_p_gain==1 && config.app.servo_pos_i_gain==7);
    command("CONFIG"); command("servo_pos_d_gain:0.75"); assert(command("APPLY")=="APPLY OK\r\n");
    eeprom.read(&loaded,0); assert(loaded.app.servo_pos_d_gain==.75f);
    assert(device.position.kd==.5f); // APPLY reloads on the actual reset, not before it.
    loaded.app.apply_servo_config(); assert(device.position.kd==.75f);
    manager.init(); manager.set_state(CommandState::RUNNING); command("TEST"); command("do_vel:0.15");
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
    for (int type : {0,2,3,6}) {
        assert(command("servo_cmd: "+std::to_string(type)+" -0.25")=="servo_cmd OK\r\n");
        assert(device.servo_type==type && device.servo_value==-.25f);
    }
    assert(write_persistent_parameter(config,ParameterId::SERVO_CONTROL_VEL_RAMP_RATE,2.0f,true)==ParameterWriteResult::OK);
    assert(command("servo_cmd: 1 0.5 255")=="servo_cmd OK\r\n");
    assert(device.servo_command->type==ServoControlType::VELOCITY_RAMP && device.servo_command->index==255);
    std::get<RampTrajectory>(*device.servo_traj_generator).reference=.25f;
    assert(apply_servo_command(1,.5f,true,255)); // Shared Serial/Cyphal dispatch.
    assert(std::get<RampTrajectory>(*device.servo_traj_generator).reference==.25f);
    auto errors_before_index=access("cmd_errors",{}).natural32.value.elements[0];
    assert(command("servo_cmd: 1 0.6 255").find("Invalid target")!=std::string::npos);
    assert(access("cmd_errors",{}).natural32.value.elements[0]==errors_before_index+1);
    assert(device.servo_command->value==.5f);
    assert(write_persistent_parameter(config,ParameterId::SERVO_CONTROL_VEL_RAMP_RATE,0.0f,true)==ParameterWriteResult::INVALID);
    assert(config.app.servo_control_vel_ramp_rate==2.0f);
    assert(command("servo_cmd: 1 0.6 0")=="servo_cmd OK\r\n");
    assert(std::get<RampTrajectory>(*device.servo_traj_generator).reference==.25f);
    command("STOP"); assert(!device.servo_command.has_value());
    assert(command("mit_cmd:\t1 2 3 4 5")=="mit_cmd OK\r\n");
    assert(device.target.angle==1 && device.target.velocity==2 && device.target.torque==3);
    assert(device.target.angle_kp==4 && device.target.velocity_kp==5);
    for (auto bad : {"mit_cmd: 1 2 3 4", "mit_cmd: 1 2 3 4 5 6", "mit_cmd: 1 2 3 bad 5",
                     "servo_cmd: 1.0 2", "servo_cmd: 7 2", "servo_cmd: -1 2", "servo_cmd: 0 2x", "servo_cmd: 0"}) {
        auto errors_before=access("cmd_errors",{}).natural32.value.elements[0];
        assert(command(bad).find("Invalid target")!=std::string::npos);
        assert(access("cmd_errors",{}).natural32.value.elements[0]==errors_before+1);
        assert(device.target.angle==1 && device.servo_value==0.0f);
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
    assert(uart_frames.size()==3 && uart_frames[0]=="servo_cmd OK\r\n");
    assert(uart_frames[1]=="STOP OK\r\n" && uart_frames[2].find("firmware_rev:")==0);
    uart_frames.clear();
    const std::string long_command="mit_cmd: 0.000000000000000000 0.000000000000000000 0 0 0\n";
    for (char byte : long_command) {receive_serial(std::string_view(&byte,1)); drain_serial();}
    assert(uart_frames.size()==1 && uart_frames[0]=="mit_cmd OK\r\n");
    uart_frames.clear();
    receive_serial(std::string(220,'x')+"\nSTOP\n"); drain_serial();
    assert(uart_frames.size()==2 && uart_frames[0].find("SERIAL ERROR:")==0 && uart_frames[1]=="STOP OK\r\n");
    uart_frames.clear();
    receive_serial(std::string(600,'x')+"\nSTOP\n"); drain_serial();
    assert(uart_frames.size()==2 && uart_frames[0].find("SERIAL ERROR:")==0 && uart_frames[1]=="STOP OK\r\n");
    // One bounded command per service, no consumption while TX owns the buffer.
    uart_frames.clear();
    receive_serial("STOP\nfirmware_rev:?\n");
    auto tail_before = serial_tail;
    huart2.gState = HAL_UART_STATE_BUSY_TX;
    serial_service();
    assert(serial_tail==tail_before && uart_frames.empty());
    huart2.gState = HAL_UART_STATE_READY;
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
    assert(serial_deferred && calibrations==1);
    uart_frames.clear();
    process_serial(); assert(!serial_deferred && calibrations==2 && uart_output=="CALIBRATE FINISH\r\n");
    assert(uart_frames.size()==2 && uart_frames[0]=="CALIBRATE OK\r\n");
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
                assert(output.find("servo_control_vel_limit:")!=std::string::npos);
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
    serial_busy=false;
    discard_serial_input(); uart_output.clear();
    receive_serial("name:?\r\nnode_id:?\r\n");
    ++test_millis; process_serial(); assert(uart_output.find("name:")==0);
    uart_output.clear(); process_serial(); assert(uart_output.empty());
    ++test_millis; process_serial(); assert(uart_output.find("node_id:")==0);
    // Explicit zero is a value, not the NAN sentinel for a default gain.
    DriveConfig zero{BASE_CONFIG_DEFAULTS, VBDriveConfig{}};
    zero.app.servo_pos_p_gain=zero.app.servo_pos_i_gain=zero.app.servo_pos_d_gain=0;
    zero.app.servo_vel_p_gain=zero.app.servo_vel_i_gain=0;
    zero.app.apply_servo_config();
    assert(device.position.kp==0 && device.position.ki==0 && device.position.kd==0);
    assert(device.velocity.kp==0 && device.velocity.ki==0);
    ParameterValue zero_value;
    assert(read_parameter(zero,ParameterId::SERVO_POS_P_GAIN,zero_value) && std::get<float>(zero_value)==0);
    // Planning tolerance is shared, persistent, live on SAVE/Cyphal, and NaN restores its default.
    manager.set_state(CommandState::RUNNING);
    command("STOP");
    assert(command("velocity_planning_tolerance:1").find("CONFIG mode required")!=std::string::npos);
    command("CONFIG");
    assert(command("velocity_planning_tolerance:1.25").find(" OK")!=std::string::npos);
    for (auto bad : {"-1", "inf", "-inf"}) {
        assert(command(std::string("velocity_planning_tolerance:")+bad).find("Invalid value")!=std::string::npos);
        assert(config.app.velocity_planning_tolerance==1.25f);
    }
    assert(command("SAVE").starts_with("SAVE OK"));
    assert(device.servo_input_config.velocity_planning_tolerance==1.25f);
    eeprom.read(&loaded,0);assert(loaded.app.velocity_planning_tolerance==1.25f);
    fill_register_real32(input,.75f);
    auto tolerance_response=access("velocity_planning_tolerance",input);
    assert(tolerance_response.real32.value.elements[0]==.75f);
    assert(device.servo_input_config.velocity_planning_tolerance==.75f);
    manager.persist_pending_config();eeprom.read(&loaded,0);assert(loaded.app.velocity_planning_tolerance==.75f);
    command("CONFIG");assert(command("velocity_planning_tolerance:nan").find(" OK")!=std::string::npos);
    command("SAVE");
    assert(command("velocity_planning_tolerance:?").find("0.500000")!=std::string::npos);
    assert(device.servo_input_config.velocity_planning_tolerance==.5f);
    eeprom.read(&loaded,0);assert(std::isnan(loaded.app.velocity_planning_tolerance));

    // Shared enable semantics across command states/transports and config transactions.
    for(auto state:{CommandState::INIT,CommandState::CONFIG,CommandState::NOT_CALIBRATED,CommandState::CALIBRATING}) {
        manager.set_state(state);device.on=false;
        assert(command("is_on:1").find("RUNNING mode required")!=std::string::npos);
        fill_register_bit(input,true);access("is_on",input);assert(!device.on);
        assert(command("is_on:0")=="is_on:0 OK\r\n");
        device.on=true;fill_register_bit(input,false);access("is_on",input);assert(!device.on);
    }
    manager.set_state(CommandState::RUNNING);
    for(float bad:{-1.f,2.f,NAN,INFINITY,-INFINITY}) {
        fill_register_real32(input,bad);access("is_on",input);assert(!device.on);
        fill_register_integer32(input,-1);access("is_on",input);assert(!device.on);
        fill_register_natural32(input,2);access("is_on",input);assert(!device.on);
    }
    fill_register_integer32(input,1);access("is_on",input);assert(device.on);
    unsigned starts=device.starts;command("is_on:1");assert(device.starts==starts);
    for(auto ending:{"EXIT","SAVE"}) {
        for(bool enabled:{false,true}) {
            command(enabled ? "is_on:1" : "is_on:0");command("CONFIG");assert(!device.on);
            command("CONFIG"); // Repeated CONFIG must retain the original enable snapshot.
            assert(command(ending).find(" OK")!=std::string::npos);assert(device.on==enabled);
        }
        command("is_on:1");command("CONFIG");command("is_on:0");command(ending);assert(!device.on);
        command("is_on:1");command("CONFIG");
        fill_register_bit(input,false);access("is_on",input);command(ending);assert(!device.on);
        command("is_on:1");command("CONFIG");device.start_result=1;
        assert(command(ending).find("ERROR: driver enable failed")!=std::string::npos);
        assert(!device.on);device.start_result=0;
    }
    command("is_on:0");
    puts("PASS: 48 shared registers, planning tolerance persistence/validation, Serial/Cyphal isolation, Servo apply and boot commands");
}
'''

with tempfile.TemporaryDirectory(prefix="vbdrive-interfaces-") as temp:
    tmp = Path(temp)
    for header in ("main.h", "stm32g4xx_hal.h", "stm32g4xx.h", "usart.h",
                   "voltbro/eeprom/eeprom.hpp", "voltbro/motors/bldc/vbdrive/vbdrive.hpp",
                   "voltbro/motors/bldc/foc/foc.hpp", "nanoprintf.h"):
        dest = tmp / header
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text('#include "stub.hpp"\n')
    (tmp / "stub.hpp").write_text(STUB)
    interface = (ROOT / "App/communications/cyphal/interface.hpp").read_text()
    callback = interface[interface.index("static void handle_parameter_register"):interface.index("void setup_subscriptions() {")]
    utils = (ROOT / "Drivers/libcxxcanard/cyphal/node/registers_utils.hpp").read_text().replace("#include <cyphal/node/registers_handler.hpp>", "")
    source = '#include "stub.hpp"\n#include "state_manager/state_manager.h"\n#include <uavcan/_register/Access_1_0.h>\n'
    source += 'using RegisterAccessResponse=uavcan_register_Access_Response_1_0;\n'
    source += r'''
VBDrive device; VBDrive* motor=&device;
VBDrive* get_motor(){return motor;}
EEPROM eeprom; EEPROM& get_eeprom(){return eeprom;}
UART_HandleTypeDef huart2;
bool is_able_to_calibrate() {return get_app_manager().get_state()==CommandState::RUNNING ||
                                  get_app_manager().get_state()==CommandState::NOT_CALIBRATED;}
bool do_calibrate() {++calibrations; return true;}
void reboot_to_bootloader(){++boots;}
#include "config/config.cpp"
#include "state_manager/state_manager.cpp"
#include "communications/serial/setup.hpp"
auto& manager = get_app_manager();
'''
    source += utils + callback + '\nvoid drain_serial(){process_serial(); for(int i=0;i<16;++i) serial_service();}\n' + TEST
    (tmp / "test.cpp").write_text(source)
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++20", "-DSTM32G4",
                    '-DVBDRIVE_FIRMWARE_REV="4.0.0"',
                    "-I"+str(tmp), "-I"+str(ROOT/"App"), "-I"+str(ROOT/"Drivers/libvoltbro"),
                    "-I"+str(ROOT/"build/Release/cyphal_types/c"), str(tmp/"test.cpp"), "-o", str(tmp/"test")], check=True)
    subprocess.run([str(tmp/"test")], check=True)
