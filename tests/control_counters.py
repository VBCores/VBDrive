#!/usr/bin/env python3
"""Check the production counters-only recorder without MCU peripherals."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='vbdrive-counters-') as directory:
 p=Path(directory)
 (p/'main.h').write_text('#pragma once\n#include <cstdint>\nusing millis=uint32_t;\n')
 (p/'app.h').write_text('#pragma once\n#include <cstdint>\nusing millis=uint32_t;\nuint32_t millis_32();\n')
 stub=p/'voltbro/motors/bldc/vbdrive/vbdrive.hpp';stub.parent.mkdir(parents=True);stub.write_text('#pragma once\n')
 code=r'''
#define COUNTERS_ONLY
#include "profiling.hpp"
#include <cassert>
uint32_t now=0;
uint32_t millis_32() {return now;}
int main() {
 for(unsigned second=1;second<=20;++second) {
  for(int call=0;call<40000;++call)profile_main_begin();
  for(int call=0;call<1000;++call) {
   VBDRIVE_PROFILE_RESULT(servo,true)
   VBDRIVE_PROFILE_COUNT(state_messages_queued)
  }
  now=second*1000;
  profile_millisecond();
  const auto& entry=counter_windows[(second-1)&15];
  assert(entry[0]==now && entry[1]==second*40000);
  assert(entry[2]==second*1000 && entry[3]==0 && entry[4]==second*1000);
 }
 assert(counter_window_count==20);
 // Missing timer callbacks do not turn a 1050-ms interval into a nominal second.
 now+=1050;VBDRIVE_PROFILE_RESULT(mit,false) profile_millisecond();
 const auto& entry=counter_windows[20&15];
 assert(entry[0]==now && entry[3]==1 && counter_window_count==21);
 profile_millisecond();assert(counter_window_count==21);
}
'''
 (p/'test.cpp').write_text(code)
 subprocess.run(['c++','-std=c++20','-O2','-I'+str(p),'-I'+str(root/'App'),'-I'+str(root/'Drivers/libvoltbro'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('PASS: counters-only rates, accepted/rejected tracking, ring wrap and timestamped delayed snapshots')
