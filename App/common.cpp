#include "app.h"

#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "voltbro/utils.hpp"

// Volatile is absolutely required here due to timer interrupt
// Otherwise HAL_Delay() will not work
static volatile uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

#ifdef MONITOR
static volatile encoder_data value_enc = 0;
volatile float value_A = 0;
volatile float value_B = 0;
volatile float value_C = 0;
volatile float value_V = 0;
volatile float value_stator_temp = 0;
volatile float value_mcu_temp = 0;
volatile float value_angle = 0;
volatile float value_velocity = 0;
volatile float value_torque = 0;

volatile float debug_torque = 0.0f;
volatile float debug_angle = 0.0f;
volatile float debug_velocity = 0.0f;
volatile float debug_angle_kp = 0.0f;
volatile float debug_velocity_kp = 0.0f;
volatile float debug_voltage = -21.0f;
volatile float debug_I_kp = 16.0f;
volatile float debug_I_ki = 0.6f;
#endif
#if defined(FOC_PROFILE) || defined(MONITOR)
volatile float value_dt = 0.0f;
#endif

#ifdef FOC_PROFILE
static bool dwt_ready = false;
static inline void init_dwt() {
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = true;
}
volatile uint32_t last_cycle_cost = 0;
volatile uint32_t value_invocations = 0;
#endif

__attribute__((hot, flatten)) void main_callback() {
    auto& app_config = get_app_config();

    #ifdef ENABLE_DT
    // it can never start at 0t, so 0 is "not set" value
    static micros last_call = 0;
    float dt = 0;
    micros now = micros_64();
    #endif
    #ifdef FOC_PROFILE
    if (!dwt_ready) {
        init_dwt();
    }
    const uint32_t start_cycles = DWT->CYCCNT;
    #endif
    #ifdef ENABLE_DT
    if (last_call != 0) {
        micros diff = subtract_64(now, last_call);
        dt = (float)diff / (float)MICROS_S;
        #ifdef MONITOR
        value_dt = dt;
        #endif
    }
    last_call = now;
    #endif

    if (app_config.is_app_running()) {
        if (auto motor = get_motor()) {
            motor->update();
        }
    }
    #ifdef FOC_PROFILE
    last_cycle_cost = DWT->CYCCNT - start_cycles;
    // <'++'/'+='/... expression of 'volatile'-qualified type is deprecated> - C++20
    value_invocations = value_invocations + 1;
    #if !defined(ENABLE_DT) && defined(MONITOR)
    value_dt = last_cycle_cost;
    #endif
    #endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        // <'++'/'+='/... expression of 'volatile'-qualified type is deprecated> - C++20
        millis_k = millis_k + 1;
    } else if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    } else if (htim->Instance == TIM4) {
        #ifdef MONITOR
        auto motor = get_motor();
        auto encoder = motor->get_encoder();
        auto inverter = static_cast<const VBInverter&>(motor->get_inverter());
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
        #endif

        main_callback();
    }
}

micros __attribute__((optimize("O0"))) micros_64() {
    return ((micros)millis_k * 1000u) + __HAL_TIM_GetCounter(&htim7);
}

micros system_time() {
    // TODO: network-wide time sync
    return micros_64();
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);
}

millis millis_32() {
    return millis_k;
}

void HAL_Delay(uint32_t delay) {
    millis wait_start = millis_32();
    while ((millis_32() - wait_start) < delay) {}
}
