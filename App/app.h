#pragma once

#include "main.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/foc/foc.h>

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 50;
inline constexpr millis DELAY_ON_ERROR_MS = 500;
std::shared_ptr<CyphalInterface> get_interface();
void in_loop_reporting(millis);
void setup_subscriptions();
void cyphal_loop();
void start_cyphal();
void set_cyphal_mode(uint8_t mode);
CanardNodeID get_node_id();

// common.cpp
micros micros_64();
millis millis_32();
void start_timers();

// app.cpp
