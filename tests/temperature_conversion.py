#!/usr/bin/env python3
"""Exercise the firmware temperature conversion with factory and ADC sample values."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] /
          'Drivers/libvoltbro/voltbro/motors/bldc/vbdrive/vbdrive.hpp').read_text()
start = source.index('    void update_temperature() {')
opening = source.index('{', start)
end = opening + 1
depth = 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
method = source[start:end]

program = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
static uint16_t vref_cal = 1657, temp_cal1 = 1031, temp_cal2 = 1367;
#define VREFINT_CAL_ADDR (&vref_cal)
#define TEMPSENSOR_CAL1_ADDR (&temp_cal1)
#define TEMPSENSOR_CAL2_ADDR (&temp_cal2)
struct Inverter {
    uint32_t ADC_1_buffer[4]{};
    uint32_t ADC_2_buffer[2]{};
    float mcu_temperature = 0, stator_temperature = 0;
''' + method + r'''
};
int main() {
    Inverter inverter;
    inverter.ADC_2_buffer[1] = 1749;
    inverter.ADC_1_buffer[2] = 998;
    inverter.ADC_1_buffer[3] = 1506;
    inverter.update_temperature();
    assert(std::fabs(inverter.mcu_temperature - 50.0f) < 1.0f);
    assert(std::fabs(inverter.stator_temperature - 31.76f) < .1f);
    assert(std::fabs(inverter.mcu_temperature + 273.15f - 323.15f) < 1.0f);

    // Same die temperature at lower VDDA: both ADC codes scale with VDDA.
    inverter.ADC_1_buffer[2] = 1030;
    inverter.ADC_1_buffer[3] = 1553;
    inverter.update_temperature();
    assert(std::fabs(inverter.mcu_temperature - 50.0f) < 1.0f);
    assert(std::fabs(inverter.stator_temperature - 31.76f) < .1f);

    inverter.ADC_1_buffer[3] = 0;
    inverter.update_temperature();
    assert(std::isnan(inverter.mcu_temperature));
    assert(std::isfinite(inverter.stator_temperature));
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-temperature-') as folder:
    cpp = Path(folder) / 'temperature.cpp'
    exe = Path(folder) / 'temperature'
    cpp.write_text(program)
    subprocess.run(['c++', '-std=c++20', '-Wall', '-Wextra', str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('Temperature conversion checks passed')
