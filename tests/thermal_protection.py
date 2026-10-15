#!/usr/bin/env python3
"""Exercise production bridge gating, calibration cancellation and Heartbeat fields.

Peripheral/DSP doubles model only hardware effects; no MCU timing claims.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / 'Drivers/libvoltbro'


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    end, depth = opening + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


vb = (LIB / 'voltbro/motors/bldc/vbdrive/vbdrive.hpp').read_text()
methods = '\n'.join(function(vb, signature) for signature in (
    'VBDriveStatus get_status()', 'ThermalState get_thermal_health()',
    'void update_thermal_protection()', 'void set_temperature_limits(',
    'HAL_StatusTypeDef init() override', 'HAL_StatusTypeDef stop() override',
    'HAL_StatusTypeDef start() override'))
gate = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <voltbro/motors/bldc/vbdrive/temperature.hpp>
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0, HAL_ERROR=1;
constexpr int LL_TIM_CHANNEL_CH1=1, LL_TIM_CHANNEL_CH2=2, LL_TIM_CHANNEL_CH3=4;
constexpr int LL_TIM_CHANNEL_CH1N=8, LL_TIM_CHANNEL_CH2N=16, LL_TIM_CHANNEL_CH3N=32;
struct Timer {bool moe=false;uint32_t channels=0;} timer;
struct Handle {Timer* Instance=&timer;} handle;
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(h) ((h)->Instance->moe=false)
#define __HAL_TIM_MOE_ENABLE(h) ((h)->Instance->moe=true)
#define CRITICAL_SECTION(code) {code}
uint32_t HAL_GetTick() {return 0;}
int HAL_TIM_Base_Start(Handle*) {assert(!timer.moe);return 0;}
void LL_TIM_CC_EnableChannel(Timer* t, unsigned mask) {assert(!t->moe);t->channels|=mask;}
struct VBInverter {
 float stator=NAN,mcu=NAN;
 void start() {assert(!timer.moe);}
 void update_temperature() {}
 float get_stator_temperature() const {return stator;}
 float get_mcu_temperature() const {return mcu;}
};
struct Sensor {void init() {assert(!timer.moe);}};
struct Encoder {int init() {assert(!timer.moe);return 0;}};
struct Regulator {int resets=0;void reset(){++resets;}};
struct Driver {int wakes=0;int wake(){++wakes;return 0;}int clear_faults(){return 0;}};
struct Base {virtual int init()=0;virtual int start()=0;virtual int stop()=0;};
struct Motor : Base {
 Handle* htim=&handle;
 VBInverter inverter;
 Encoder encoder;Sensor inductive_sensor;Driver gate_driver;
 Regulator q_reg,d_reg;
 bool _is_on=false,history=true;
 float bus_current=1,voltage=1;
 uint32_t bootstrap_charge_deadline_ms=0;
 static constexpr uint32_t bootstrap_charge_time_ms=5;
 float max_stator_temperature=DEFAULT_MAX_STATOR_TEMPERATURE_K;
 float restart_stator_temperature=DEFAULT_RESTART_STATOR_TEMPERATURE_K;
 bool stator_hot=false;
 VBDriveStatus thermal_status=VBDriveStatus::COOLING_DOWN;
 bool is_on()const{return _is_on;}
 void reset_servo_input(){history=false;}
 bool set_voltage_point(float v){voltage=v;return true;}
 void force_bootstrap_charge(){bus_current=0;}
 void quit_stall(){}
''' + methods + r'''
};
int main() {
 Motor m;
 assert(m.get_status()==VBDriveStatus::COOLING_DOWN);
 assert(m.init()==HAL_OK && !timer.moe && timer.channels==63);
 assert(m.start()==HAL_ERROR && !timer.moe && m.gate_driver.wakes==0);
 m.inverter.mcu=30;m.inverter.stator=25;
 assert(m.start()==HAL_OK && timer.moe && m.is_on());
 assert(!m.history && m.voltage==0 && m.bus_current==0);
 m.history=true;m.voltage=2;
 m.inverter.stator=90;m.update_thermal_protection();
 assert(!timer.moe && !m.is_on() && !m.history && m.voltage==0);
 assert(m.q_reg.resets==2 && m.d_reg.resets==2);
 assert(m.start()==HAL_ERROR && !timer.moe);
 m.inverter.stator=80;m.update_thermal_protection();assert(m.get_thermal_health()==ThermalState::CAUTION);
 m.inverter.stator=60;assert(m.start()==HAL_OK && timer.moe && m.get_thermal_health()==ThermalState::ADVISORY);
 m.inverter.mcu=131;m.update_thermal_protection();assert(!timer.moe);
 m.inverter.mcu=130;assert(m.start()==HAL_OK);
 m.inverter.stator=NAN;m.update_thermal_protection();assert(!timer.moe);
 m.inverter.stator=75;assert(m.start()==HAL_OK && m.get_thermal_health()==ThermalState::ADVISORY);
 m.stop();assert(!timer.moe);
}
'''

calibration = (LIB / 'voltbro/motors/bldc/foc/calibration.cpp').read_text()
body = function(calibration, 'bool FOC::calibrate(')
circular = calibration[calibration.index('template <class Container'):calibration.index('// NOTE: caller')]
cal = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <tuple>
#include <voltbro/math/consts.h>
using encoder_data=uint16_t;
using float32_t=float;
constexpr size_t CALIBRATION_BUFF_SIZE=2048;
using __non_const_calib_array_t=std::array<int,CALIBRATION_BUFF_SIZE>;
using calibration_array_t=const __non_const_calib_array_t;
__CALIBRATION_RECORD__
struct arm_fir_instance_f32 {};
void arm_fir_init_f32(arm_fir_instance_f32*,int,float*,float*,int){}
void arm_fir_f32(arm_fir_instance_f32*,float* in,float* out,int n){std::copy_n(in,n,out);}
unsigned now=0,last_check=0,max_spacing=0,done=0,stop_at=0,writes_after_trip=0;
bool tripped=false;
void HAL_Delay(uint32_t t){assert(t<=10);now+=t;}
void Error_Handler(){assert(false);}
struct FOC {
 struct {int CPR=16384,value=0;int get_value()const{return value;}} encoder;
 struct {float calibration_voltage=.2f;struct {int ppairs=14,gear_ratio=10;} common;} drive_info;
 float angle=0,raw_rotor_angle=0;int DQs[3]{};bool on=true;
 bool set_voltage_point(float){return true;}
 int stop(){on=false;return 0;}
 void set_pwm(){if(tripped)++writes_after_trip;}
 void set_windings_calibration(float a){assert(on);angle=a;set_pwm();}
 void update_angle(){
  float turns=angle/(TWO_PI*drive_info.common.ppairs);
  encoder.value=(static_cast<int>(std::round(turns*encoder.CPR))%encoder.CPR+encoder.CPR)%encoder.CPR;
  raw_rotor_angle=encoder.value;
 }
 bool calibrate(CalibrationData&,std::byte*,size_t,void (*)(int,int),bool (*)());
};
void progress(int d,int total){assert(d==int(done+1) && total==10);done=d;}
bool check(){
 max_spacing=std::max(max_spacing,now-last_check);last_check=now;
 if(done>=stop_at){tripped=true;return false;}
 return true;
}
''' + circular + body + r'''
int main(){
 static_assert(CalibrationData::TYPE_ID==0x89ABCDF0);
 CalibrationData record;record.reset();assert(!record.was_calibrated && record.type_id==0x89ABCDF0);
 alignas(16) std::array<std::byte,16384> buffer;
 for(stop_at=0;stop_at<=11;++stop_at){
  FOC m;CalibrationData data;
  now=last_check=max_spacing=done=writes_after_trip=0;tripped=false;
  bool success=m.calibrate(data,buffer.data(),buffer.size(),progress,check);
  assert(success==(stop_at==11));
  assert(max_spacing<=10 && writes_after_trip==0);
  if(!success)assert(!m.on && done==stop_at);
  else assert(done==10);
 }
}
'''

foc=(LIB/'voltbro/motors/bldc/foc/foc.hpp').read_text()
record=function(foc,'struct __attribute__((packed)) CalibrationData')+';'
cal=cal.replace('__CALIBRATION_RECORD__','#define USE_CALIBRATION_ARRAY\n'+record)

heartbeat_source=(ROOT/'App/communications/cyphal/setup.hpp').read_text()
heartbeat=r'''
#include <cassert>
#include <cstdint>
#include <cmath>
#include <voltbro/motors/bldc/vbdrive/temperature.hpp>
using CanardTransferID=uint8_t;
constexpr uint8_t uavcan_node_Health_1_0_CAUTION=2;
constexpr uint8_t uavcan_node_Health_1_0_NOMINAL=0,uavcan_node_Health_1_0_ADVISORY=1;
constexpr unsigned uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_=7509,MICROS_S=1000000;
struct HBeat {uint32_t uptime;struct{uint8_t value;} health,mode;uint8_t vendor_specific_status_code;};
uint8_t CYPHAL_MODE=0;bool _is_cyphal_on=true;
unsigned millis_32(){return 1000;}
template<class E> auto to_underlying(E e){return static_cast<uint8_t>(e);}
struct Motor {VBDriveStatus status=VBDriveStatus::NOMINAL;ThermalState health=ThermalState::NOMINAL;
 auto get_status()const{return status;}auto get_thermal_health()const{return health;}} motor;
Motor* ptr=&motor;Motor* get_motor(){return ptr;}
struct Interface {HBeat last{};void send_msg(HBeat* p,unsigned,CanardTransferID*,unsigned){last=*p;}} interface;
Interface* cyphal_interface=&interface;
''' + function(heartbeat_source,'void heartbeat()') + r'''
int main(){
 for(ThermalState state : {ThermalState::NOMINAL,ThermalState::ADVISORY,ThermalState::CAUTION}){
  const auto health=to_underlying(state);
  motor.health=state;motor.status=state==ThermalState::CAUTION?VBDriveStatus::COOLING_DOWN:VBDriveStatus::NOMINAL;
  heartbeat();assert(interface.last.health.value==health);
  assert(interface.last.vendor_specific_status_code==(health==2?1:0));
 }
 ptr=nullptr;heartbeat();assert(interface.last.health.value==2 && interface.last.vendor_specific_status_code==0);
}
'''
heartbeat=heartbeat.replace('#include <cmath>','#include <cmath>\n#include <initializer_list>')
app=(ROOT/'App/app.cpp').read_text()
orchestration=r"""
#include <cassert>
#include <cstddef>
#include <cstdint>
constexpr int HAL_OK=0;
#define HAL_IMPORTANT(expr) assert((expr)==0);
enum class VBDriveStatus {NOMINAL,COOLING_DOWN};
enum class CommandState {RUNNING,CALIBRATING,NOT_CALIBRATED};
struct CalibrationData {bool was_calibrated=false;void reset(){was_calibrated=false;}} calibration_data;
struct EEPROM {unsigned writes=0;template<class T> int write(T*,unsigned){++writes;return 0;}} eeprom;
unsigned paused=0,resumed=0,discarded=0;
constexpr unsigned CALIBRATION_PLACEMENT=0x100,SHARED_BUFFER_SIZE=16384;
std::byte cyphal_queue_buffer_shared[SHARED_BUFFER_SIZE];
void report_calibration_progress(int,int){}
void pause_cyphal_for_calibration(){++paused;}
void resume_cyphal_after_calibration(){++resumed;}
void discard_serial_input(){++discarded;}
struct FOCTarget{float torque;};
struct Motor {
 bool on=false,trip=false,applied=false;
 VBDriveStatus status=VBDriveStatus::NOMINAL;
 void reset_servo_input(){}
 bool set_foc_point(FOCTarget){return true;}
 bool is_on()const{return on;}
 int set_state(bool enabled){on=enabled;return 0;}
 auto get_status()const{return status;}
 bool calibrate(CalibrationData&,std::byte*,unsigned,void(*)(int,int),bool(*check)()){
  if(trip)status=VBDriveStatus::COOLING_DOWN;
  return check();
 }
 void apply_calibration(CalibrationData&){applied=true;}
} device;
Motor* motor=&device;
struct Manager {
 CommandState state=CommandState::RUNNING;
 bool restore=true;
 void set_state(CommandState s){state=s;}
 void service_temperature(){if(device.status==VBDriveStatus::COOLING_DOWN)device.on=false;}
 int set_motor_enabled(bool enable){restore=false;return device.set_state(enable);}
} manager;
auto& get_app_manager(){return manager;}
"""+function(app,'bool do_calibrate()')+r"""
int main(){
 for(bool trip:{false,true})for(bool was_on:{false,true}){
  device=Motor{};device.on=was_on;device.trip=trip;
  eeprom.writes=paused=resumed=discarded=0;manager=Manager{};
  const bool success=do_calibrate();
  assert(success==!trip && paused==1 && resumed==1 && discarded==1);
  assert(eeprom.writes==(trip?0:1) && device.applied==!trip);
  assert(device.on==(!trip && was_on));
  assert(manager.state==(trip?CommandState::NOT_CALIBRATED:CommandState::RUNNING));
  if(trip)assert(!manager.restore && !calibration_data.was_calibrated);
 }
}
"""
orchestration=orchestration.replace('#include <cstddef>','#include <cstddef>\n#include <initializer_list>')

with tempfile.TemporaryDirectory(prefix='vbdrive-thermal-') as directory:
    folder=Path(directory)
    for name,source in [('gate',gate),('calibration',cal),('heartbeat',heartbeat),('orchestration',orchestration)]:
        cpp=folder/(name+'.cpp');exe=folder/name;cpp.write_text(source)
        subprocess.run(['c++','-std=c++20','-O2','-DMONITOR','-I'+str(LIB),str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
print('PASS: thermal bridge gate, calibration cancellation at every stage, Heartbeat fields')
