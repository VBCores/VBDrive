#!/usr/bin/env python3
"""Host checks of the production Kalman observer: initialization, wrapping and tracking."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
base = root / 'Drivers/libvoltbro/voltbro/motors/bldc/foc'
header = (base / 'foc.hpp').read_text()
source = (base / 'foc.cpp').read_text()
state = header[header.index('    struct FilterState'):header.index('    float elec_angle')]
body = source[source.index('void FOC::apply_kalman()'):source.index('void FOC::update_sensors()')]
code = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include "voltbro/profiling.hpp"
#define pi2 6.28318530718f
#define PI 3.14159265358979f
float mfmod(float x,float y) { return x-int(x/y)*y; }
struct FOC {
 float raw_rotor_angle=0, elec_angle=0, shaft_velocity=0, shaft_angle=0, T=.000025f;
 struct {float expected_a=0,g1=.015700989410003974f,g2=3.925227776360174f,g3=387.54711795263574f;} filters_config;
 struct {struct {int ppairs=14,gear_ratio=36;} common;} drive_info;
''' + state + 'void apply_kalman();\nvoid update_shaft_angle();\n};\n' + body + r'''
float phase_error(float a,float b) { return std::remainder(a-b,pi2); }
int main() {
 // Shaft position deliberately uses the encoder before the state filter.
 FOC shaft;
 shaft.shaft_velocity=42;
 for(float unwrapped : {1.f,2.f,3.f,4.f,5.f,6.f,6.4f,6.f,5.f}) {
  shaft.raw_rotor_angle=std::fmod(unwrapped,pi2);
  shaft.filter_state.rotor_angle=shaft.raw_rotor_angle+2; // Must not affect reported position.
  shaft.update_shaft_angle();
  assert(std::fabs(shaft.shaft_angle-unwrapped/36)<1e-6f);
  assert(shaft.shaft_velocity==42);
 }
 // Arbitrary starting phases must not manufacture motion, including near the wrap.
 for(float angle : {0.f,1.f,3.5f,6.28f}) {
  FOC m; m.raw_rotor_angle=angle;
  for(int k=0;k<1000;++k) {
   m.apply_kalman();
   assert(std::fabs(m.shaft_velocity)<1e-7f);
   assert(std::fabs(phase_error(m.elec_angle,14*angle))<1e-5f);
  }
 }
 // Interleaved instances must retain independent states.
 FOC first,second; first.raw_rotor_angle=1; second.raw_rotor_angle=2;
 for(int k=0;k<100;++k){first.apply_kalman();second.apply_kalman();}
 assert(first.filter_state.rotor_angle==1 && second.filter_state.rotor_angle==2);
 // Both directions and constant acceleration cross many 2*pi boundaries.
 for(int scenario=0;scenario<3;++scenario) {
  FOC m;
  for(int k=0;k<40000;++k) {
   double t=k*.000025;
   double theta=scenario==0?36*t:scenario==1?-36*t:50*t*t;
   double velocity=scenario==0?1:scenario==1?-1:100*t/36;
   m.raw_rotor_angle=std::fmod(theta, double(pi2));
   if(m.raw_rotor_angle<0)m.raw_rotor_angle+=pi2;
   m.apply_kalman();
   if(k>20000) {
    assert(std::fabs(phase_error(m.elec_angle, std::fmod(14*theta,double(pi2))))<.001f);
    assert(std::fabs(m.shaft_velocity-velocity)<.002f);
   }
  }
 }
 // A known acceleration is added once; the residual acceleration starts at zero.
 FOC m; m.filters_config.expected_a=100; m.apply_kalman();
 m.raw_rotor_angle=.5f*100*m.T*m.T; m.apply_kalman();
 assert(std::fabs(m.filter_state.residual_acceleration)<1e-7f);
 assert(std::fabs(m.shaft_velocity-100*m.T/36)<1e-7f);
 puts("Kalman initialization, instance isolation, wrap and tracking checks passed");
}
'''
code = '#include <initializer_list>\n' + code
with tempfile.TemporaryDirectory(prefix='vbdrive-kalman-') as directory:
    path = Path(directory)
    (path / 'test.cpp').write_text(code)
    subprocess.run(['clang++', '-std=c++17', '-O2', '-I'+str(root/'Drivers/libvoltbro'), str(path/'test.cpp'), '-o', str(path/'test')], check=True)
    subprocess.run([str(path/'test')], check=True)
