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

formula = next(line.strip() for line in source.splitlines() if line.strip().startswith('bus_current ='))
program = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "voltbro/math/consts.h"
#define FORCE_INLINE inline
#define CRITICAL_SECTION(code) {code}
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer) moe=false
using HAL_StatusTypeDef=int;
constexpr int HAL_OK=0;
struct Base { virtual int stop() {return 0;} };
struct Regulator {void reset() {}};
struct Motor:Base {
 float bus_current=0;
 bool _is_on=true,moe=true;
 int* htim=nullptr;
 uint32_t bootstrap_charge_deadline_ms=0;
 unsigned DQs[3]={1,2,3};
 Regulator q_reg,d_reg;
 void reset_servo_input() {}
 bool set_voltage_point(float) {return true;}
 void set_pwm() {assert(DQs[0]==0 && DQs[1]==0 && DQs[2]==0);}
 void estimate(float V_d,float V_q,float I_D,float I_Q,float busV) {
''' + formula + '\n}\n' + function(header, 'float get_bus_current() const') + '\n' + function(board, 'FORCE_INLINE void force_bootstrap_charge()') + '\n' + function(board, 'HAL_StatusTypeDef stop() override') + r'''
};
''' + function(transform, 'inline void abc(') + r'''
void near(float a,float b) {assert(std::abs(a-b)<1e-5f);}
int main() {
 Motor m,other;
 m.estimate(0,12,0,4,24); near(m.get_bus_current(),3);
 near(other.get_bus_current(),0); // Per-motor estimate, not shared state.
 m.estimate(0,12,0,-4,24); near(m.get_bus_current(),-3);
 m.estimate(4,6,2,-1,24); near(m.get_bus_current(),.125f);
 m.estimate(0,0,2,4,24); near(m.get_bus_current(),0);
 m.estimate(12,12,4,4,0); near(m.get_bus_current(),0);
 m.estimate(12,12,4,4,-1); near(m.get_bus_current(),0);
 // Rotating both vectors leaves total three-phase power and bus current unchanged.
 for(unsigned i=0;i<72;++i) {
  float theta=TWO_PI*i/72,u,v,w,a,b,c;
  abc(std::sin(theta),std::cos(theta),4,6,&u,&v,&w);
  abc(std::sin(theta),std::cos(theta),2,-1,&a,&b,&c);
  m.estimate(4,6,2,-1,24);
  near(m.get_bus_current()*24,u*a+v*b+w*c);
 }
 m.estimate(0,12,0,4,24); m.force_bootstrap_charge(); near(m.get_bus_current(),0);
 m.estimate(0,12,0,4,24); assert(m.stop()==HAL_OK);
 assert(!m._is_on && !m.moe); near(m.get_bus_current(),0);
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-bus-current-') as directory:
    p = Path(directory)
    (p / 'test.cpp').write_text(program)
    subprocess.run(['c++','-std=c++20','-O2','-I'+str(root/'Drivers/libvoltbro'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
print('PASS: dq bus-current estimate, regeneration, three-phase power, zero bus, bootstrap and OFF')
