#!/usr/bin/env python3
"""Check the production bus-current expression, power identity and disabled output."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
base = root / 'Drivers/libvoltbro/voltbro/motors/bldc'
source = (base / 'foc/foc.cpp').read_text()
header = (base / 'foc/foc.hpp').read_text()
board = (base / 'vbdrive/vbdrive.hpp').read_text()
transform = (root / 'Drivers/libvoltbro/voltbro/math/transform.hpp').read_text()

def function(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

# Extract both the measurement and publication paths, including production PWM clipping.
sample_start = source.index('    const float sampled_bus_current =')
sample = source[sample_start:source.index(';', sample_start) + 1]
estimate_start = source.index('    bus_current =')
estimate = source[estimate_start:source.index(';', estimate_start) + 1]
pwm_start = source.index('    float v_u =')
pwm = source[pwm_start:source.index('    set_pwm();', pwm_start) + len('    set_pwm();')]
assert sample_start < pwm_start < estimate_start
program = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include "voltbro/math/consts.h"
#define FORCE_INLINE inline
#define CRITICAL_SECTION(code) {code}
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer) moe=false
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0;
float fminf3(float a,float b,float c){return std::min(a,std::min(b,c));}
float fmaxf3(float a,float b,float c){return std::max(a,std::max(b,c));}
''' + function(transform, 'inline void abc(') + function(transform, 'inline void svm(') + r'''
struct Base { virtual int stop() {return 0;} virtual void set_pwm() {} };
struct Regulator {void reset() {}};
struct Motor:Base {
 float bus_current=0;
 bool _is_on=true,moe=true;
 struct Timer {unsigned CCR1=0,CCR2=0,CCR3=0;} timer;
 struct Handle {Timer* Instance;} handle{&timer};
 Handle* htim=&handle;
 uint32_t bootstrap_charge_deadline_ms=0;
 uint16_t DQs[3]={0,0,0};
 static constexpr int full_pwm=1999;
''' + next(line.strip() for line in board.splitlines() if 'static constexpr uint16_t max_pwm_compare =' in line) + r'''
 struct {float I_lpf_coefficient=1;} filters_config;
 struct {
  float a=0,b=0,c=0;
  float get_A()const{return a;} float get_B()const{return b;} float get_C()const{return c;}
 } inverter;
 Regulator q_reg,d_reg;
 void reset_servo_input() {}
 bool set_voltage_point(float) {return true;}
''' + function(board, 'FORCE_INLINE void set_pwm() override') + r'''
 void program_pwm(float V_d,float V_q,float busV,float theta) {
  struct {float s,c;} elec_angles{std::sin(theta),std::cos(theta)};
''' + pwm + r'''
 }
 void tick(float busV,float next_vq=0,float theta=0) {
''' + sample + r'''
  program_pwm(0,next_vq,busV>0?busV:24,theta);
''' + estimate + r'''
 }
 void currents(float id,float iq,float theta) {
  abc(std::sin(theta),std::cos(theta),id,iq,&inverter.a,&inverter.b,&inverter.c);
 }
''' + function(header, 'float get_bus_current() const') + '\n' + function(board, 'FORCE_INLINE void force_bootstrap_charge()') + '\n' + function(board, 'HAL_StatusTypeDef stop() override') + r'''
};
void near(float a,float b,float tolerance=1e-5f) {assert(std::abs(a-b)<tolerance);}
int main() {
 Motor m,other;
 // Linear modulation agrees with dq power, apart from integer PWM quantization.
 for(unsigned i=0;i<72;++i) {
  const float theta=TWO_PI*i/72;
  m.program_pwm(4,6,24,theta);m.currents(2,-1,theta);
  m.tick(24,24,theta);near(m.get_bus_current(),.125f,.003f);
 }
 near(other.get_bus_current(),0); // Per-motor estimate, not shared state.
 m.program_pwm(0,4,24,0);m.currents(0,2,0);
 m.tick(24);near(m.get_bus_current(),.5f,.002f);
 m.program_pwm(0,4,24,0);m.currents(0,-2,0);
 m.tick(24);near(m.get_bus_current(),-.5f,.002f);
 // Clipping at 88% must reduce power. This formerly reported 15 A.
 m.program_pwm(0,24,24,0);m.currents(0,10,0);
 assert(m.timer.CCR1==1000 && m.timer.CCR2==1760 && m.timer.CCR3==0);
 m.tick(24,0);near(m.get_bus_current(),7.6210236f);
 // New zero target does not retroactively change the sampled interval's duty.
 assert(m.DQs[0]==1000 && m.DQs[1]==1000 && m.DQs[2]==1000);
 m.tick(24,24);near(m.get_bus_current(),0);
 // Compare clipped power through a complete electrical revolution, both signs.
 for(int sign : {-1,1}) for(unsigned i=0;i<360;++i) {
  const float theta=TWO_PI*i/360;
  m.program_pwm(0,24,24,theta);m.currents(0,sign*10,theta);
  float a,b,c;
  abc(std::sin(theta),std::cos(theta),0,sign*10,&a,&b,&c);
  const float ua=24*m.timer.CCR1/2000.f,ub=24*m.timer.CCR2/2000.f,uc=24*m.timer.CCR3/2000.f;
  const float power=ua*a+ub*b+uc*c;
  m.tick(24,0,theta);near(m.get_bus_current()*24,power,.0001f);
  assert(sign*m.get_bus_current()>7.5f && sign*m.get_bus_current()<8.81f);
 }
 // Scalar LPF averages signed power, not separate voltage/current products.
 m.bus_current=0;m.filters_config.I_lpf_coefficient=.25f;m.currents(0,10,0);
 for(int i=0;i<4;++i){m.DQs[0]=1000;m.DQs[1]=1760;m.DQs[2]=0;m.tick(24);}
 near(m.get_bus_current(),7.6210236f*(1-std::pow(.75f,4)));
 m.tick(0);near(m.get_bus_current(),0);
 m.bus_current=3;m.tick(-1);near(m.get_bus_current(),0);
 m.bus_current=3;m.force_bootstrap_charge();near(m.get_bus_current(),0);
 assert(m.timer.CCR1==0 && m.timer.CCR2==0 && m.timer.CCR3==0);
 m.bus_current=3;assert(m.stop()==HAL_OK);
 assert(!m._is_on && !m.moe);near(m.get_bus_current(),0);
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-bus-current-') as directory:
    p = Path(directory)
    (p / 'test.cpp').write_text(program)
    subprocess.run(['c++','-std=c++20','-O2','-I'+str(root/'Drivers/libvoltbro'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
print('PASS: clipped PWM bus current, sampled interval, signed power/LPF, zero bus, bootstrap and OFF')
