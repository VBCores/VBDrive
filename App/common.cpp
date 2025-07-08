#include "app.h"

#include "tim.h"

static uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

#define DEBUG
#ifdef DEBUG
static volatile encoder_data value_enc = 0;
static volatile float value_A = 0;
static volatile float value_B = 0;
static volatile float value_C = 0;
static volatile float value_V = 0;
static volatile float value_stator_temp = 0;
static volatile float value_mcu_temp = 0;
static volatile float debug_voltage = 0;
#endif

void main_callback() {
    auto& app_config = get_app_config();

    if (app_config.is_app_running()) {
        auto motor = get_motor();
        motor->update();
        cyphal_loop();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    else if (htim->Instance == TIM1) {
        main_callback();

        #ifdef DEBUG
        auto motor = get_motor();
        motor->update_sensors();
        auto encoder = motor->get_encoder();
        auto inverter = static_cast<const VBInverter&>(motor->get_inverter());
        value_enc = encoder.get_value();
        value_A = inverter.get_A();
        value_B = inverter.get_B();
        value_C = inverter.get_C();
        value_V = inverter.get_busV();
        value_stator_temp = inverter.get_stator_temperature();
        value_mcu_temp = inverter.get_mcu_temperature();
        motor->set_voltage_point(debug_voltage);
        #endif
    }
    else if (htim->Instance == TIM6) {
        millis_k += 1;
    }
}

micros __attribute__((optimize("O0"))) micros_64() {
    return ((micros)millis_k * 1000u) + __HAL_TIM_GetCounter(&htim6);
}

millis millis_32() {
    return millis_k;
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_Delay(uint32_t delay) {
    millis wait_start = millis_32();
    while ((millis_32() - wait_start) < delay) {}
}
