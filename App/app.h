#pragma once

#include <cstdint>

class VBDrive;
class EEPROM;

VBDrive* get_motor();
EEPROM& get_eeprom();
uint64_t system_time();
uint64_t micros_64();
uint32_t millis_32();
void start_timers();
void monitor_loop(uint32_t now);
bool is_able_to_calibrate();
bool do_calibrate();
void reboot_to_bootloader();
