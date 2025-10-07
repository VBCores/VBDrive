#include "app.h"

#include "tim.h"

// Volatile is absolutely required here due to timer interrupt
// Otherwise HAL_Delay() will not work
static volatile uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

#ifdef MONITOR
static volatile encoder_data value_enc = 0;
static volatile float value_A = 0;
static volatile float value_B = 0;
static volatile float value_C = 0;
static volatile float value_V = 0;
static volatile float value_stator_temp = 0;
static volatile float value_mcu_temp = 0;
static volatile float value_angle = 0;
static volatile float value_velocity = 0;
static volatile float value_torque = 0;


static volatile float debug_torque = 0.0f;
static volatile float debug_angle = 0.0f;
static volatile float debug_velocity = 0.0f;
static volatile float debug_angle_kp = 0.0f;
static volatile float debug_velocity_kp = 0.0f;
static volatile float debug_voltage = -21.0f;
static volatile float debug_dt = 0.0f;
#endif

inline void main_callback() {
    auto& app_config = get_app_config();

    // it can never start at 0t, so 0 is "not set" value
    static micros last_call = 0;
    float dt = 0;
    micros now = micros_64();
    if (last_call != 0) {
        micros diff = subtract_64(now, last_call);
        dt = (float)diff / (float)MICROS_S;
        #ifdef MONITOR
        debug_dt = dt;
        #endif
    }
    last_call = now;

    if (app_config.is_app_running()) {
        static millis last_cyphal_call = 0;
        EACH_N(millis_k, last_cyphal_call, 2, {
            cyphal_loop();
        })
        get_motor()->update_with_dt(dt);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        millis_k += 1;
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
        }
        #endif

        main_callback();
    }
}

micros __attribute__((optimize("O0"))) micros_64() {
    return ((micros)millis_k * 1000u) + __HAL_TIM_GetCounter(&htim6);
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

millis millis_32() {
    return millis_k;
}

void HAL_Delay(uint32_t delay) {
    millis wait_start = millis_32();
    while ((millis_32() - wait_start) < delay) {}
}
