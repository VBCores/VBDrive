#!/usr/bin/env python3
"""Check production compare IRQ and VBDrive update ordering with peripheral doubles."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
it = (root / 'Core/Src/stm32g4xx_it.c').read_text()
irq = it[it.index('void TIM1_CC_IRQHandler(void)'):]
irq = irq[:irq.index('\n}') + 2]
source = (root / 'Drivers/libvoltbro/voltbro/motors/bldc/vbdrive/vbdrive.hpp').read_text()
update = source[source.index('        void update() override {'):source.index('        // for logging')]
utils = (root / 'Drivers/libvoltbro/voltbro/utils.hpp').read_text()
macro = utils[utils.index('#define EACH_N('):utils.index('#define EACH_N_MICROS')]
program = r'''
#include <cassert>
#include <cstdint>
#include <vector>
std::vector<int> events;
struct FOC {
 virtual void update() { events.push_back(1); }
 void update_sensors() { events.push_back(2); }
};
uint32_t HAL_GetTick() {return 0;}
struct Sensor {void update() {events.push_back(4);}};
''' + macro + r'''
struct Motor : FOC {
 bool _is_on=false, bootstrap=false;
 Sensor inductive_sensor;
 bool is_bootstrap_charging(uint32_t) {return bootstrap;}
 void force_bootstrap_charge() {events.push_back(3);}
''' + update + r'''
};
constexpr int RESET=0, TIM_FLAG_CC4=16, TIM_IT_CC4=16;
int htim1=0, flags=0, enabled=0, callbacks=0, fallbacks=0;
#define __HAL_TIM_GET_FLAG(t,b) (flags & (b))
#define __HAL_TIM_GET_IT_SOURCE(t,b) (enabled & (b))
#define __HAL_TIM_CLEAR_IT(t,b) (flags &= ~(b))
void main_callback() {assert(!(flags & TIM_FLAG_CC4)); ++callbacks;}
void HAL_TIM_IRQHandler(int*) {++fallbacks;}
''' + irq + r'''
int main() {
 // A compare flag alone must not run FOC, nor may another event run it.
 flags=16; enabled=0; TIM1_CC_IRQHandler(); assert(callbacks==0);
 flags=1; enabled=16; TIM1_CC_IRQHandler(); assert(callbacks==0);
 flags=17; TIM1_CC_IRQHandler(); assert(callbacks==1 && flags==1);
 TIM1_CC_IRQHandler(); assert(callbacks==1 && fallbacks==3);
 Motor motor;
 for (unsigned tick=0; tick<90; ++tick) {
  // Exercise all paths; encoder cadence must continue across transitions.
  motor._is_on=tick%3!=0; motor.bootstrap=tick%3==1;
  events.clear(); motor.update();
  assert(events[0]==(tick%3==2 ? 1 : 2));
  unsigned count=tick%3==1 ? 2 : 1;
  if(tick%3==1) assert(events[1]==3);
  if(tick && tick%9==0) {assert(events.back()==4); ++count;}
  assert(events.size()==count);
 }
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-pwm-sync-') as directory:
    path = Path(directory)
    (path / 'test.cpp').write_text(program)
    subprocess.run(['c++', '-std=c++20', '-O2', str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)

# Timing contract between the CubeMX configuration and peripheral setup.
ioc = dict(line.split('=', 1) for line in (root / 'VBDrive.ioc').read_text().splitlines() if '=' in line)
clock = int(ioc['RCC.APB2TimFreq_Value']) / (int(ioc['TIM1.Prescaler']) + 1)
period = int(ioc['TIM1.PeriodNoDither'])
compare = int(ioc['TIM1.PulseNoDither_4'])
assert clock / (period + 1) == 40000
assert abs((period - compare) / clock - 4e-6) < 1e-12
assert ioc['TIM1.CounterMode'] == 'TIM_COUNTERMODE_DOWN'
assert ioc['TIM1.TIM_MasterOutputTrigger'] == 'TIM_TRGO_UPDATE'
tim = (root / 'Core/Src/tim.c').read_text()
assert f'sConfigOC.Pulse = {compare};' in tim
assert 'TIM_OCMODE_TIMING' in tim
assert 'HAL_NVIC_EnableIRQ(TIM4_IRQn)' not in tim
app = (root / 'App/app.cpp').read_text()
assert 'HAL_TIM_Base_Start_IT(&htim4)' not in app
assert '__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);' in app
assert len(re.findall(r'\bmain_callback\(\);', it)) == 1
print('PASS: compare IRQ gating/acknowledgment, 40 kHz/4 us configuration, encoder tail ordering and cadence in ON/OFF/bootstrap')
