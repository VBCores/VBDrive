#pragma once
#include "main.h"
#include "app.h"
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/profiling.hpp>

#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE) || defined(COUNTERS_ONLY)
inline volatile uint32_t servo_commands_accepted = 0;
inline volatile uint32_t servo_commands_rejected = 0;
inline volatile uint32_t mit_commands_accepted = 0;
inline volatile uint32_t mit_commands_rejected = 0;
inline volatile uint32_t motor_callback_count = 0;
inline volatile uint32_t cyphal_loop_invocations = 0;
inline volatile uint32_t state_messages_queued = 0;
#endif
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
using HandlerTimingStats = profiling::IntervalStats;
inline HandlerTimingStats servo_handler_timing;
inline HandlerTimingStats mit_handler_timing;
inline HandlerTimingStats can_arrival_timing;
#endif

#ifdef MONITOR
inline volatile encoder_data value_enc = 0;
inline volatile float value_A = 0;
inline volatile float value_B = 0;
inline volatile float value_C = 0;
inline volatile float value_V = 0;
inline volatile float value_stator_temp = 0;
inline volatile float value_mcu_temp = 0;
inline volatile float value_angle = 0;
inline volatile float value_velocity = 0;
inline volatile float value_torque = 0;

inline volatile float debug_torque = 0.0f;
inline volatile float debug_angle = 0.0f;
inline volatile float debug_velocity = 0.0f;
inline volatile float debug_angle_kp = 0.0f;
inline volatile float debug_velocity_kp = 0.0f;
inline volatile float debug_voltage = -21.0f;
inline volatile float debug_I_kp = 16.0f;
inline volatile float debug_I_ki = 0.6f;
#endif
#if defined(FOC_PROFILE) || defined(MONITOR)
inline volatile float value_dt = 0.0f;
#endif

#ifdef FOC_PROFILE
#ifndef FOC_PROFILE_SAMPLE_PERIOD
#define FOC_PROFILE_SAMPLE_PERIOD 256u
#endif
inline volatile uint32_t last_cycle_cost = 0, max_cycle_cost = 0, value_invocations = 0;
inline uint16_t profile_sample_counter = 0;
#endif
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
inline profiling::RollingFrequency<2> rolling_frequency;
inline volatile uint32_t rolling_invocations_per_second = 0;
inline volatile uint32_t rolling_min_invocations_per_second = UINT32_MAX;
inline volatile uint32_t rolling_window_count = 0;
inline volatile uint32_t rolling_cyphal_loops_per_second = 0;
inline volatile uint32_t rolling_min_cyphal_loops_per_second = UINT32_MAX;

inline void profile_handler(profiling::IntervalStats& stats) {
    static const uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    const uint32_t now_ms = millis_32();
    const uint32_t now_cycles = profiling::cycles();
    stats.record(now_ms, now_cycles, cycles_per_ms, motor_callback_count, cyphal_loop_invocations);
}
#define VBDRIVE_PROFILE_HANDLER(stats) profile_handler(stats)
#else
#define VBDRIVE_PROFILE_HANDLER(stats)
#endif
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE) || defined(COUNTERS_ONLY)
#define VBDRIVE_PROFILE_COUNT(counter) counter = counter + 1;
#define VBDRIVE_PROFILE_RESULT(kind, accepted) do { \
    if (accepted) { VBDRIVE_PROFILE_COUNT(kind##_commands_accepted) } \
    else { VBDRIVE_PROFILE_COUNT(kind##_commands_rejected) } \
} while (false);
#else
#define VBDRIVE_PROFILE_COUNT(counter)
#define VBDRIVE_PROFILE_RESULT(kind, accepted)
#endif

#ifdef COUNTERS_ONLY
// Cumulative snapshots: millis, FOC calls, accepted, rejected, State queued, comms passes.
// Read outside the trial; timestamps distinguish full active seconds from idle edges.
inline volatile uint32_t counter_windows[16][6]{};
inline volatile uint32_t counter_window_count = 0;
#endif

inline void profile_start() {
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE) || defined(FOC_PROFILE_DETAILED)
    profiling::init_cycles();
#endif
}

inline profiling::Sample profile_main_begin() {
    VBDRIVE_PROFILE_COUNT(motor_callback_count)
#ifdef ENABLE_DT
    static micros last_call = 0;
    const micros now = micros_64();
#endif
#ifdef FOC_PROFILE
    const auto sample = profiling::sample_cycles(profile_sample_counter, FOC_PROFILE_SAMPLE_PERIOD);
#else
    const profiling::Sample sample{};
#endif
#ifdef ENABLE_DT
#ifdef MONITOR
    if (last_call != 0) value_dt = float(subtract_64(now, last_call)) / float(MICROS_S);
#endif
    last_call = now;
#endif
    return sample;
}

inline void profile_main_end([[maybe_unused]] profiling::Sample sample) {
#ifdef FOC_PROFILE
    if (sample.selected) {
        last_cycle_cost = profiling::cycles() - sample.start;
        if (last_cycle_cost > max_cycle_cost) max_cycle_cost = last_cycle_cost;
        value_invocations = value_invocations + FOC_PROFILE_SAMPLE_PERIOD;
#if !defined(ENABLE_DT) && defined(MONITOR)
        value_dt = last_cycle_cost;
#endif
    }
#endif
}

inline void profile_millisecond() {
#ifdef COUNTERS_ONLY
    static uint32_t previous_millis = 0;
    const uint32_t now = millis_32();
    if (now - previous_millis >= 1000) {
        previous_millis = now;
        auto& entry = counter_windows[counter_window_count & 15U];
        entry[0] = 0;
        entry[1] = motor_callback_count;
        entry[2] = servo_commands_accepted + mit_commands_accepted;
        entry[3] = servo_commands_rejected + mit_commands_rejected;
        entry[4] = state_messages_queued;
        entry[5] = cyphal_loop_invocations;
        entry[0] = now;
        counter_window_count = counter_window_count + 1;
    }
#endif
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
    if (rolling_frequency.record({motor_callback_count, cyphal_loop_invocations})) {
        rolling_invocations_per_second = rolling_frequency.rate[0];
        rolling_cyphal_loops_per_second = rolling_frequency.rate[1];
        rolling_min_invocations_per_second = rolling_frequency.minimum[0];
        rolling_min_cyphal_loops_per_second = rolling_frequency.minimum[1];
        rolling_window_count = rolling_frequency.windows;
    }
#endif
}

#ifdef STACK_PROFILE
extern uint32_t __StackLimit, __StackTop;
inline volatile size_t max_stack_usage = 0;
#endif
inline void profile_mark_stack() {
#ifdef STACK_PROFILE
    uint32_t sp;
    __asm__ volatile ("mov %0, sp" : "=r" (sp));
    profiling::mark_stack(&__StackLimit, reinterpret_cast<uint32_t*>(sp));
#endif
}

inline void monitor_loop([[maybe_unused]] millis current_t) {
#ifdef STACK_PROFILE
    static millis stack_time = 0;
    if (current_t - stack_time >= 100) {
        stack_time = current_t;
        const size_t used = profiling::stack_usage(&__StackLimit, &__StackTop);
        if (used > max_stack_usage) max_stack_usage = used;
    }
#endif
#ifdef MONITOR
    static millis monitor_time = 0;
    EACH_N(current_t, monitor_time, 1, {
        auto motor = get_motor();
        const auto& encoder = motor->get_encoder();
        const auto& inverter = static_cast<const VBInverter&>(motor->get_inverter());
        value_angle = motor->get_angle();
        value_velocity = motor->get_velocity();
        value_torque = motor->get_torque();
        value_enc = encoder.get_value();
        value_A = inverter.get_A();
        value_B = inverter.get_B();
        value_C = inverter.get_C();
        value_V = inverter.get_busV();
        value_stator_temp = inverter.get_stator_temperature();
        value_mcu_temp = inverter.get_mcu_temperature();

        if (debug_voltage > -10.0f) {
            motor->set_voltage_point(debug_voltage);
        }
        else if (debug_voltage > -20.0f){
            motor->set_foc_point(FOCTarget{
                .torque = debug_torque,
                .angle = debug_angle,
                .velocity = debug_velocity,
                .angle_kp = debug_angle_kp,
                .velocity_kp = debug_velocity_kp,
            });
            motor->set_current_regulator_params(debug_I_kp, debug_I_ki);
        }
    })
#endif
}

