#!/usr/bin/env python3
"""Production TIM7 polling clock: do not count a tick already serviced by an IRQ."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'App/app.cpp').read_text();start=s.index('millis millis_32() {');end=s.index('\n}',start)+2
function=s[start:end]
u=(root/'Drivers/libvoltbro/voltbro/utils.hpp').read_text();macro=u[u.index('#define CRITICAL_SECTION'):u.index('// TODO: add optional warning')]
program=r'''
#include <cassert>
#include <cstdint>
using millis=uint32_t;
volatile millis millis_k=0;
int htim7;
constexpr int TIM_FLAG_UPDATE=1,RESET=0;
bool flag=false,interleave=false;
unsigned mask=0;
unsigned __get_PRIMASK(){return mask;}
void __disable_irq(){mask=1;}
void __set_PRIMASK(unsigned m){mask=m;}
bool get_flag(){
 bool observed=flag;
 if(interleave){assert(mask==0);interleave=false;flag=false;millis_k=millis_k+1;}
 return observed;
}
#define __HAL_TIM_GET_FLAG(timer,bit) get_flag()
#define __HAL_TIM_CLEAR_FLAG(timer,bit) flag=false
'''+macro+function+r'''
int main(){
 millis_k=100;flag=true;interleave=true;
 assert(millis_32()==101);assert(mask==0);assert(millis_32()==101);
 flag=true;assert(millis_32()==102);assert(!flag && mask==0);
 flag=true;mask=1;assert(millis_32()==103);assert(mask==1);
 millis_k=UINT32_MAX;flag=true;assert(millis_32()==0);assert(mask==1);
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-clock-') as d:
 p=Path(d);(p/'test.cpp').write_text(program)
 subprocess.run(['c++','-std=c++20','-O2',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('PASS: pending TIM7 tick counted once across IRQ interleaving, PRIMASK preserved and wrap handled')
