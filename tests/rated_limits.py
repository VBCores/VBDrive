#!/usr/bin/env python3
"""Exercise production BLDC rating normalization, target checks and stall limits."""
from pathlib import Path
import subprocess,tempfile,re
root=Path(__file__).resolve().parents[1]
bldc=(root/'Drivers/libvoltbro/voltbro/motors/bldc/bldc.h').read_text()
impl=(root/'Drivers/libvoltbro/voltbro/motors/bldc/bldc.cpp').read_text()
impl=re.sub(r'//[^\n]*','',impl)
base=(root/'Drivers/libvoltbro/voltbro/motors/motor_commons.hpp').read_text()
def function(s,name):
 start=s.index(name);a=s.index('{',start);n=1;b=a+1
 while n:n+=(s[b]=='{')-(s[b]=='}');b+=1
 return s[start:b]
code='''
#include <cassert>
#include <cmath>
#include <algorithm>
#define FORCE_INLINE inline
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0;
struct DriveRuntimeConfig {
 float user_current_limit=NAN,user_torque_limit=NAN,current_limit=NAN;
 float user_position_lower_limit=NAN,user_position_upper_limit=NAN;
 float user_speed_limit=NAN;int user_angle_direction=1;
};
struct AbstractMotor {
 DriveRuntimeConfig drive_runtime_config;
'''
for sig in ['virtual HAL_StatusTypeDef apply_runtime_config()', 'virtual bool check_runtime_config(', 'bool set_runtime_config(']:code+=function(base,sig)+'\n'
code+='''};
struct BLDCController:AbstractMotor {
 struct {float max_current=2,max_torque=7,torque_const=.5f,stall_current=6,stall_timeout=3,stall_tolerance=.2f;struct {int gear_ratio=2;} common;} drive_info;
 bool is_stalling=false;float shaft_velocity=0;
 void detect_stall(double);void quit_stall();
'''
code+=bldc[bldc.index('    FORCE_INLINE bool is_symmetric_limit_set'):bldc.index('    FORCE_INLINE float get_direction_multiplier')]
code+=function(bldc,'virtual bool check_runtime_config(')+'\n'+function(bldc,'virtual HAL_StatusTypeDef apply_runtime_config()')+'\n};\n'
code+=function(impl,'void BLDCController::detect_stall(')+'\n'+function(impl,'void BLDCController::quit_stall()')
code+='''
int main(){
 BLDCController m;DriveRuntimeConfig requested;
 requested.user_current_limit=20;requested.user_torque_limit=40;
 assert(m.set_runtime_config(requested));
 assert(requested.user_current_limit==20 && requested.user_torque_limit==40);
 assert(m.drive_runtime_config.user_current_limit==2 && m.drive_runtime_config.user_torque_limit==7);
 assert(m.drive_runtime_config.current_limit==2);
 for(int gear:{1,10,36}) {
  m.drive_info.common.gear_ratio=gear;
  assert(m.is_torque_target_valid(1) && m.is_torque_target_valid(-1));
  assert(!m.is_torque_target_valid(1.01f) && !m.is_torque_target_valid(-1.01f));
 }
 m.detect_stall(0);m.detect_stall(4);
 assert(m.is_stalling && m.drive_runtime_config.current_limit==2); // Stall rating 6 must not raise 2.
 requested.user_current_limit=1;requested.user_torque_limit=3;requested.current_limit=6;
 assert(m.set_runtime_config(requested) && m.drive_runtime_config.current_limit==1);
 m.quit_stall();assert(m.drive_runtime_config.current_limit==1);
 requested.user_current_limit=20;requested.user_torque_limit=40;
 assert(m.set_runtime_config(requested) && m.drive_runtime_config.current_limit==2);
 m.drive_info.max_current=20;m.drive_info.max_torque=7;
 assert(m.set_runtime_config(requested));
 assert(m.is_torque_target_valid(7) && !m.is_torque_target_valid(7.01f)); // Torque rating is now tighter.
 requested={};assert(m.set_runtime_config(requested));
 assert(m.drive_runtime_config.current_limit==20 && m.drive_runtime_config.user_torque_limit==7);
 requested.user_current_limit=.5f;requested.user_torque_limit=1;
 assert(m.set_runtime_config(requested));
 assert(m.drive_runtime_config.current_limit==.5f && !m.is_torque_target_valid(.26f));
 m.detect_stall(0);m.detect_stall(4);assert(m.drive_runtime_config.current_limit==.5f);
 m.quit_stall();assert(m.drive_runtime_config.current_limit==.5f);
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-rated-') as d:
 p=Path(d);(p/'test.cpp').write_text(code)
 subprocess.run(['c++','-std=c++20','-O2',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('PASS: rated priority, larger user settings, effective limits, torque checks, updates and stall enter/exit')
