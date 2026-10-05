#!/usr/bin/env python3
"""Exercise shared EEPROM schemas, bootloader prefix/baud loading and profiling math."""
from pathlib import Path
import subprocess
import tempfile
import math

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / 'Drivers/libvoltbro'


def function(text, signature):
    start = text.index(signature + ' {')
    opening = text.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + '\n'


config = r'''
#include <array>
#include <cassert>
#include <cstring>
#include <cstdint>
class EEPROM {
public:
 std::array<uint8_t,16384> bytes;
 unsigned writes=0;
 bool fail=false;
 template<class T> int read(T* p,uint16_t address) {
  if(fail)return 1;
  assert(address+sizeof(T)<=bytes.size()); memcpy(p,bytes.data()+address,sizeof(T));return 0;
 }
 template<class T> int write(T* p,uint16_t address) {
  if(fail)return 1;
  assert(address+sizeof(T)<=bytes.size());memcpy(bytes.data()+address,p,sizeof(T));++writes;return 0;
 }
};
#include "config/config.hpp"
'''
config += function((ROOT / 'App/config/config.cpp').read_text(),
                   'bool VBDriveConfig::are_required_params_set(const BaseConfigData& base) const')
config += r'''
struct Address32Storage {
 DriveConfig value{};
 template<class T> int read(T* out,uint32_t address) {assert(address==0x08060000);memcpy(out,&value,sizeof(T));return 0;}
 template<class T> int write(T* in,uint32_t address) {assert(address==0x08060000);memcpy(&value,in,sizeof(T));return 0;}
};
int main() {
 for(uint32_t address : {0U,0x40U,0x3000U}) {
  EEPROM memory;memory.bytes.fill(0xA5);
  DriveConfigManager placed(memory,BASE_CONFIG_DEFAULTS,address);
  assert(placed.load() && placed.save());
  placed.get_config().base.node_id=1;placed.get_config().app.gear_ratio=36;
  placed.mark_changed();assert(placed.save());
  placed.begin();placed.get_config().app.kp=8;placed.mark_changed();assert(placed.commit());
  placed.get_committed_config().base.serial_baud=230400;placed.mark_committed_changed();assert(placed.save_pending());
  assert(placed.load());assert(placed.get_config().app.kp==8 && placed.get_config().base.serial_baud==230400);
  for(size_t i=0;i<memory.bytes.size();++i)
   if(i<address || i>=address+sizeof(DriveConfig))assert(memory.bytes[i]==0xA5);
 }
 Address32Storage flash;
 ConfigManager<VBDriveConfig,Address32Storage> wide(flash,BASE_CONFIG_DEFAULTS,0x08060000);
 assert(wide.load());wide.mark_changed();assert(wide.save());
 EEPROM e; e.bytes.fill(0xA5);
 DriveConfigManager m(e,BASE_CONFIG_DEFAULTS);
 assert(m.load() && m.needs_save());
 assert(!m.get_config().are_required_params_set());
 assert(m.get_config().app.gear_ratio==0);
 assert(parameter_default<uint32_t>(ParameterId::GEAR)==36);
 assert(m.save() && e.writes==1);
 for(size_t i=sizeof(DriveConfig);i<e.bytes.size();++i)assert(e.bytes[i]==0xA5);
 auto& cfg=m.get_config();cfg.base.node_id=1;cfg.app.gear_ratio=36;
 strcpy(cfg.base.name,"axis-01");m.mark_changed();assert(m.save());
 const BaseConfigData saved_base=cfg.base;
 // An unknown application layout must retain communications and device identity.
 cfg.app.type_id=0xDEADBEEF;e.write(&cfg,0);
 assert(m.load() && m.needs_save());
 assert(memcmp(&cfg.base,&saved_base,sizeof(saved_base))==0);
 assert(cfg.app.type_id==VBDriveConfig::TYPE_ID && cfg.app.gear_ratio==0);
 assert(!cfg.are_required_params_set());assert(m.save());
 cfg.app.gear_ratio=36;m.mark_changed();assert(m.save());
 // Transport writes during CONFIG must persist the committed copy, never staged values.
 m.begin();cfg.app.kp=9;m.mark_changed();
 m.get_committed_config().app.servo_pos_i_gain=7;m.mark_committed_changed();
 assert(m.save_pending());DriveConfig disk;e.read(&disk,0);
 assert(std::isnan(disk.app.kp) && disk.app.servo_pos_i_gain==7);
 m.discard();assert(std::isnan(cfg.app.kp) && cfg.app.servo_pos_i_gain==7);
 m.begin();cfg.base.serial_baud=230400;cfg.app.kp=9;m.mark_changed();
 const unsigned writes=e.writes;assert(m.commit() && e.writes==writes+1);
 e.read(&disk,0);assert(disk.base.serial_baud==230400 && disk.app.kp==9);
 m.begin();m.reset();assert(cfg.base.serial_baud==115200 && cfg.app.gear_ratio==0);
 m.discard();assert(cfg.base.serial_baud==230400 && cfg.app.gear_ratio==36);
 // An invalid common block is reset independently of a valid application block.
 cfg.base.type_id=0;e.write(&cfg,0);assert(m.load());
 assert(cfg.base.type_id==VOLTBRO_BASE_CONFIG_TYPE_ID && cfg.base.node_id==0);
 assert(cfg.app.gear_ratio==36 && cfg.app.kp==9);
 e.fail=true;assert(!m.save() && m.needs_save());assert(!m.load());
 for(unsigned baud:{9600,19200,38400,57600,115200,230400,460800,921600,1000000})assert(voltbro_serial_baud_valid(baud));
 for(unsigned baud:{0u,1u,9601u,UINT32_MAX})assert(!voltbro_serial_baud_valid(baud));
 for(size_t i=sizeof(DriveConfig);i<e.bytes.size();++i)assert(e.bytes[i]==0xA5);
}
'''

profiling = r'''
#include <cassert>
#include "voltbro/profiling.hpp"
int main() {
 profiling::IntervalStats s;
 s.record(1,UINT32_MAX-499,1000,40,10);
 s.record(2,500,1000,80,20);
 assert(s.stream_count==2 && s.min_gap_cycles==1000 && s.max_gap_cycles==1000);
 assert(s.elapsed_lo==1000 && s.elapsed_hi==0 && s.gap_histogram[10]==1);
 assert(s.first_counter_delta==40 && s.second_counter_delta==10);
 s.record(503,1000,1000,120,30);assert(s.stream_starts==2 && s.stream_count==1);
 profiling::IntervalStats long_run;
 for(uint32_t i=0;i<60000;++i)long_run.record(i,uint32_t(uint64_t(i)*160000),160000,40*i,10*i);
 const uint64_t elapsed=(uint64_t(long_run.elapsed_hi)<<32)|long_run.elapsed_lo;
 assert(elapsed==uint64_t(59999)*160000 && long_run.gap_histogram[10]==59999);
 profiling::RollingFrequency<2> rate;
 for(unsigned i=1;i<=2100;++i)rate.record({40*i,10*i});
 assert(rate.rate[0]==40000 && rate.minimum[0]==40000 && rate.rate[1]==10000 && rate.windows==11);
 std::array<uint32_t,32> stack{};
 profiling::mark_stack(stack.data(),stack.data()+24);
 assert(profiling::stack_usage(stack.data(),stack.data()+32)==8);
 stack[20]=1;assert(profiling::stack_usage(stack.data(),stack.data()+32)==12);
 uint16_t count=0;for(int i=1;i<=512;++i)assert(profiling::sample_cycles(count,256).selected==(i%256==0));
 VB_PROFILE_COUNT(undefined_disabled_counter)
 VB_PROFILE_BEGIN(undefined_disabled_timer)
 VB_PROFILE_END(undefined_disabled_maximum, undefined_disabled_timer)
}
'''

boot = r'''
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "voltbro/config/base_config.h"
#define BOOTLOADER_USE_EEPROM_CONFIG
#define CONFIG_EEPROM_I2C_DEV_ADDR 0x50U
#define CONFIG_EEPROM_MEM_ADDR VB_CONFIG_ADDRESS
#define I2C_MEMADD_SIZE_16BIT 2U
#define HAL_OK 0
static uint8_t memory[16384];
static bool ready=true,g_diag_ready=false;
static uint32_t last_baud;
static int hi2c2;
typedef struct {uint32_t can_id;uint8_t nominal_prescaler,data_prescaler;bool fd_mode,bitrate_switch;} BootTransportConfig;
static int HAL_I2C_IsDeviceReady(void* p,unsigned dev,unsigned tries,unsigned timeout) {(void)p;(void)dev;(void)tries;(void)timeout;return ready?0:1;}
static int HAL_I2C_Mem_Read(void* p,unsigned dev,unsigned address,unsigned mode,uint8_t* out,unsigned size,unsigned timeout) {
 (void)p;(void)mode;(void)timeout;assert(dev==0xA0 && address==CONFIG_EEPROM_MEM_ADDR && size==sizeof(BaseConfigData));memcpy(out,memory+address,size);return 0;
}
static void boot_diag_usart2_init(uint32_t baud) {last_baud=baud;}
'''
boot_source = (ROOT / 'VBBoot/App/app.c').read_text()
for signature in ('static bool eeprom_config_read(BaseConfigData* config)',
                  'static bool is_valid_config_node_id(uint8_t node_id)',
                  'static void transport_config_force_fd_brs(BootTransportConfig* config)',
                  'static bool transport_config_load_eeprom(BootTransportConfig* config)',
                  'void boot_diag_init(void)'):
    boot += function(boot_source, signature)
boot += r'''
int main(void) {
 BaseConfigData base={VOLTBRO_BASE_CONFIG_TYPE_ID,230400,7,4,3,1,"axis-07"};
 memcpy(memory+CONFIG_EEPROM_MEM_ADDR,&base,sizeof(base));memset(memory+CONFIG_EEPROM_MEM_ADDR+sizeof(base),0xF1,sizeof(memory)-CONFIG_EEPROM_MEM_ADDR-sizeof(base));
 BootTransportConfig transport={0};
 assert(transport_config_load_eeprom(&transport));
 assert(transport.can_id==7 && transport.nominal_prescaler==1 && transport.data_prescaler==1 && transport.fd_mode && transport.bitrate_switch);
 boot_diag_init();assert(last_baud==230400 && g_diag_ready);
 for(unsigned nominal=0;nominal<5;++nominal)for(unsigned data=0;data<4;++data){
  base.fdcan_nominal_baud=nominal;base.fdcan_data_baud=data;memcpy(memory+CONFIG_EEPROM_MEM_ADDR,&base,sizeof(base));
  assert(transport_config_load_eeprom(&transport));
  assert(transport.nominal_prescaler==(16U>>nominal) && transport.data_prescaler==(8U>>data));
 }
 base.serial_baud=123;memcpy(memory+CONFIG_EEPROM_MEM_ADDR,&base,sizeof(base));
 assert(!transport_config_load_eeprom(&transport));boot_diag_init();assert(last_baud==115200);
 base.serial_baud=9600;base.node_id=0;memcpy(memory+CONFIG_EEPROM_MEM_ADDR,&base,sizeof(base));
 assert(!transport_config_load_eeprom(&transport));boot_diag_init();assert(last_baud==9600);
 base.type_id=0x44AAAC01;memcpy(memory+CONFIG_EEPROM_MEM_ADDR,&base,sizeof(base));
 assert(!transport_config_load_eeprom(&transport));boot_diag_init();assert(last_baud==115200);
 ready=false;assert(!transport_config_load_eeprom(&transport));
}
'''

with tempfile.TemporaryDirectory(prefix='vbdrive-contracts-') as directory:
    tmp = Path(directory)
    for name, source, compiler, standard, extension in (
        ('config', config, 'c++', 'c++20', 'cpp'),
        ('profiling', profiling, 'c++', 'c++20', 'cpp'),
        ('boot-0', '#define VB_CONFIG_ADDRESS 0\n'+boot, 'cc', 'c11', 'c'),
        ('boot-40', '#define VB_CONFIG_ADDRESS 0x40\n'+boot, 'cc', 'c11', 'c'),
        ('boot-3000', '#define VB_CONFIG_ADDRESS 0x3000\n'+boot, 'cc', 'c11', 'c'),
    ):
        path = tmp / (name + '.' + extension)
        path.write_text(source)
        subprocess.run([compiler, '-std='+standard, '-O2', '-Wall', '-Wextra',
                        '-I'+str(LIB), '-I'+str(ROOT/'App'), str(path), '-o', str(tmp/name)], check=True)
        subprocess.run([str(tmp/name)], check=True)
        print('PASS:', name)

    # Read defaults from the production constexpr catalog, including shared C constants.
    catalog = tmp / 'catalog.cpp'
    catalog.write_text(r'''
#include "config/config.hpp"
#include <iostream>
#include <iomanip>
int main() {
 for (const auto& parameter : PARAMETER_CATALOG) {
  if (!parameter.default_value) continue;
  std::cout << parameter.name << '\t';
  std::visit([](auto value) { std::cout << std::setprecision(9) << value; }, *parameter.default_value);
  std::cout << '\n';
 }
}
''')
    subprocess.run(['c++', '-std=c++20', '-I'+str(LIB), '-I'+str(ROOT/'App'),
                    str(catalog), '-o', str(tmp/'catalog')], check=True)
    defaults = subprocess.check_output([str(tmp/'catalog')], text=True)
    rows = {}
    for line in (ROOT/'README.md').read_text().splitlines():
        if not line.startswith('| `'): continue
        cells = [cell.strip() for cell in line.split('|')[1:-1]]
        rows.setdefault(cells[0].strip('`'), cells[-1].split()[0].strip('`'))
    for line in defaults.splitlines():
        name, value = line.split('\t')
        documented = rows[name]
        if name == 'name': assert value == documented
        else:
            actual, expected = float(value), float(documented)
            assert (math.isnan(actual) and math.isnan(expected)) or math.isclose(actual, expected, rel_tol=1e-6), name
    print('PASS: README defaults match the production parameter catalog')
