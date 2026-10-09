#define NANOPRINTF_IMPLEMENTATION
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS   1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS       1 // float
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS       1 // 'l' (long), 'll' (long long)
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS       1 // 'hh' (char), 'h' (short)
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS      0 // %b (binary)
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS   0 // %n
#include "nanoprintf.h"

#include "app.h"
#include "config/config.hpp"
#include "state_manager/state_manager.h"
#include "profiling.hpp"
#include "communications/serial/setup.hpp"
#include "communications/cyphal/setup.hpp"
#include "communications/cyphal/interface.hpp"
#include <memory>
#include <type_traits>
#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "spi.h"
#include "cordic.h"
#include <voltbro/devices/stspin32g4.hpp>
#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
#include "stm32g4xx_hal_tim.h"

#ifndef NDEBUG
// Keep failed preconditions inspectable without newlib's allocating stdio path.
const char* volatile assertion_file = nullptr;
const char* volatile assertion_expression = nullptr;
volatile int assertion_line = 0;
extern "C" [[noreturn]] void __assert_func(const char* file, int line, const char*, const char* expression) {
    assertion_file = file;
    assertion_line = line;
    assertion_expression = expression;
    if (htim1.Instance != nullptr) __HAL_TIM_MOE_DISABLE(&htim1);
    Error_Handler();
    for (;;) {}
}
#endif

static constexpr uint32_t SETUP_NODE_ID_MAX = CANARD_NODE_ID_MAX - 1U;

static CanardNodeID make_unconfigured_setup_node_id() {
    uint32_t hash = 2166136261UL;
    const uint32_t uid_words[3] = {
        HAL_GetUIDw0(),
        HAL_GetUIDw1(),
        HAL_GetUIDw2()
    };
    for (uint32_t word : uid_words) {
        for (uint8_t byte_index = 0; byte_index < 4; byte_index++) {
            hash ^= (word >> (byte_index * 8U)) & 0xFFU;
            hash *= 16777619UL;
        }
    }
    return static_cast<CanardNodeID>(1U + (hash % SETUP_NODE_ID_MAX));
}

extern "C" {
    // Setup allocates Cyphal shared ownership; runtime cannot grow the heap.
    bool global_allocation_lock = false;
}

void setup_cordic() {
    CORDIC_ConfigTypeDef cordic_config {
        .Function = CORDIC_FUNCTION_COSINE,
        .Scale = CORDIC_SCALE_0,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
        .NbWrite = CORDIC_NBWRITE_1,
        .NbRead = CORDIC_NBREAD_2,
        .Precision = CORDIC_PRECISION_6CYCLES
    };
    HAL_IMPORTANT(HAL_CORDIC_Configure(&hcordic, &cordic_config));
}

EEPROM eeprom(&hi2c2, 64, I2C_MEMADD_SIZE_16BIT);
EEPROM& get_eeprom() {
    return eeprom;
}
static STSPIN32G4 motor_gate_driver(&hi2c3, GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin));

// correct elec_offset will be set by apply_calibration
AS5047P motor_encoder(GpioPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin), &hspi1);
InductiveSensor inductive_sensor(
    eeprom,
    IND_SENSOR_STATE_PLACEMENT,
    &hspi3,
    GpioPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin)
);
VBInverter motor_inverter(&hadc1, &hadc2);
alignas(4) static CalibrationData calibration_data;  // avoid stack overflow and misalignment issues for I2C EEPROM
static std::aligned_storage_t<sizeof(VBDrive), alignof(VBDrive)> motor_storage;
static VBDrive* motor = nullptr;
VBDrive* get_motor() {
    return motor;
}

void create_motor(VBDriveConfig& config_data) {
    constexpr float voltage_limit = 50.0f;
    motor = new (&motor_storage) VBDrive(
        0.000025f,
        // Kalman filter for determining electric angle
        FiltersConfig {
            .expected_a = value_or_default(config_data.filter_a, parameter_default<float>(ParameterId::FLT_A)),
            .g1 = value_or_default(config_data.filter_g1, parameter_default<float>(ParameterId::FLT_G1)),
            .g2 = value_or_default(config_data.filter_g2, parameter_default<float>(ParameterId::FLT_G2)),
            .g3 = value_or_default(config_data.filter_g3, parameter_default<float>(ParameterId::FLT_G3)),
            .I_lpf_coefficient = value_or_default(config_data.I_lpf_coefficient, parameter_default<float>(ParameterId::I_LPF))
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, parameter_default<float>(ParameterId::KP)),
            .ki = value_or_default(config_data.ki, parameter_default<float>(ParameterId::KI)),
            .kd = value_or_default(config_data.kd, parameter_default<float>(ParameterId::KD)),
            .integral_error_lim = voltage_limit,
            .max_output = voltage_limit,
            .min_output = -voltage_limit,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, parameter_default<float>(ParameterId::KP)),
            .ki = value_or_default(config_data.ki, parameter_default<float>(ParameterId::KI)),
            .kd = value_or_default(config_data.kd, parameter_default<float>(ParameterId::KD)),
            .integral_error_lim = voltage_limit,
            .max_output = voltage_limit,
            .min_output = -voltage_limit,
        },
        // User-defined runtime config
        DriveRuntimeConfig {
            .user_current_limit = value_or_default(config_data.max_current, NAN),
            .user_torque_limit = value_or_default(config_data.max_torque, NAN),
            .user_speed_limit = value_or_default(config_data.max_speed, NAN),
            .user_position_lower_limit = value_or_default(config_data.min_angle, NAN),
            .user_position_upper_limit = value_or_default(config_data.max_angle, NAN),
            .user_angle_offset = value_or_default(config_data.angle_offset, parameter_default<float>(ParameterId::ANG_OFF)),
            .user_angle_direction = vbdrive_direction_multiplier(config_data.angle_direction)
        },
        // Built-in constant parameters
        DriveInfo {
            .torque_const = value_or_default(config_data.torque_const, parameter_default<float>(ParameterId::KT)),
            .max_current = value_or_default(config_data.rated_max_current, parameter_default<float>(ParameterId::RATED_MAX_CURRENT)),
            .max_torque = value_or_default(config_data.rated_max_torque, parameter_default<float>(ParameterId::RATED_MAX_TORQUE)),
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.2f,
            .en_pin = GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin),
            .common = {
                .ppairs = 14,
                .gear_ratio = value_or_default(
                    config_data.gear_ratio,
                    static_cast<uint8_t>(parameter_default<uint32_t>(ParameterId::GEAR)),
                    static_cast<uint8_t>(0)
                )
            }
        },
        &htim1,
        motor_encoder,
        motor_inverter,
        motor_gate_driver,
        inductive_sensor,
        config_data.angle_encoder
    );
    config_data.apply_servo_config();
    HAL_Delay(100);
    motor->init();
}

bool is_able_to_calibrate() {
    auto& app_manager = get_app_manager();
    auto state = app_manager.get_state();
    return (
        state == CommandState::NOT_CALIBRATED ||
        state == CommandState::RUNNING
    );
}

static void report_calibration_progress(int done, int total) {
    serial_send_message("CALIBRATE %d/%d DONE\r\n", done, total);
}

bool do_calibrate() {
    // Stop all control and enable the bridge for the calibration motion.
    motor->reset_servo_input();
    motor->set_foc_point(FOCTarget{0});
    const bool was_on = motor->is_on();
    if (!was_on && motor->set_state(true) != HAL_OK) return false;
    auto& app_manager = get_app_manager();
    app_manager.set_state(CommandState::CALIBRATING);
    discard_serial_input();

    pause_cyphal_for_calibration();
    calibration_data.reset();
    motor->calibrate(calibration_data, cyphal_queue_buffer_shared, SHARED_BUFFER_SIZE, report_calibration_progress);
    calibration_data.was_calibrated = true;
    HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    motor->apply_calibration(calibration_data);

    const bool stopped = was_on || motor->set_state(false) == HAL_OK;
    app_manager.set_state(CommandState::RUNNING);
    resume_cyphal_after_calibration();
    return stopped;
}

void apply_calibration() {
    if (calibration_data.type_id == 0) {  // uninitialized, try to read from EEPROM
        HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    }
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        auto& app_manager = get_app_manager();
        char warning_message[] = "Motor is not calibrated! Movement forbidden\n\r\0";
        serial_send_message(warning_message);
        app_manager.set_state(CommandState::NOT_CALIBRATED);
        motor->stop();
        return;
    }
    motor->apply_calibration(calibration_data);
}

void app() {
    profile_mark_stack();
    start_timers();
    eeprom.wait_until_available();
    auto& app_manager = get_app_manager();
    app_manager.init();
    start_uart_recv_it();
    auto& config_data = app_manager.get_config();
    if (!app_manager.is_app_running() && config_data.are_required_params_set()) {
        config_data.base.was_configured = true;
        app_manager.set_state(CommandState::RUNNING);
    }

    if (!app_manager.is_app_running()) {
        // Bring up Cyphal even with blank EEPROM so config can be restored over CAN.
        if (config_data.base.node_id == 0) {
            config_data.base.node_id = make_unconfigured_setup_node_id();
        }
        start_cyphal();
        set_cyphal_mode(uavcan_node_Mode_1_0_MAINTENANCE);
        while (true) {
            process_serial();
            cyphal_loop();
            get_app_manager().persist_pending_config();
            reboot_to_bootloader_if_requested();
            if (app_manager.is_app_running()) {
                HAL_NVIC_SystemReset();
            }
        }
    }

    setup_cordic();
    create_motor(config_data.app);
    motor->start();
    apply_calibration();
    motor->set_foc_point(FOCTarget{0});

    // Warm up
    for (uint8_t i = 0; i < 32; ++i) {
        motor->update();
    }

    start_cyphal();
    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Lock heap, no dynamic memory is used at runtime
    global_allocation_lock = true;

    // TIM1 already runs PWM/ADC. Enable only the internal compare interrupt:
    // HAL_TIM_OC_Start_IT would also enable MOE and energize the power stage.
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

    while(true) {
        cyphal_loop();
        process_serial();
        get_app_manager().persist_pending_config();
        reboot_to_bootloader_if_requested();

        millis current_time = millis_32();
        monitor_loop(current_time);

    }
}

#define BOOT_REQUEST_MAGIC 0xB00710ADUL
void reboot_to_bootloader() {
    auto motor = get_motor();

    if (motor != nullptr) {
        motor->set_foc_point(FOCTarget{0});
        motor->stop();
    }

    (void)HAL_FDCAN_Stop(&hfdcan1);

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    TAMP->BKP1R = 0U;
    TAMP->BKP2R = 0U;
    HAL_PWR_DisableBkUpAccess();

    HAL_Delay(50);
    NVIC_SystemReset();
}

// Volatile is absolutely required here due to timer interrupt
// Otherwise HAL_Delay() will not work
static volatile uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

extern "C" __attribute__((hot)) void main_callback() {
    auto& app_manager = get_app_manager();
    const auto sample = profile_main_begin();
    // Service operations pause motor updates while thread mode changes hardware/config.
    if (app_manager.is_app_running() && !serial_deferred) {
        if (auto motor = get_motor()) motor->update();
    }
    profile_main_end(sample);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        // <'++'/'+='/... expression of 'volatile'-qualified type is deprecated> - C++20
        millis_k = millis_k + 1;
        profile_millisecond();
    } else if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
}

micros __attribute__((optimize("O0"))) micros_64() {
    return ((micros)millis_32() * 1000u) + __HAL_TIM_GetCounter(&htim7);
}

micros system_time() {
    // TODO: network-wide time sync
    return micros_64();
}

void start_timers() {
    profile_start();
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);
}

millis millis_32() {
    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) == RESET) return millis_k;
    millis value;
    // The TIM7 IRQ or a higher-priority poller may consume this tick before entry.
    CRITICAL_SECTION({
        if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
            millis_k = millis_k + 1;
        }
        value = millis_k;
    })
    return value;
}

void HAL_Delay(uint32_t delay) {
    millis wait_start = millis_32();
    while ((millis_32() - wait_start) < delay) {}
}

static_assert(CALIBRATION_PLACEMENT + sizeof(CalibrationData) <= IND_SENSOR_STATE_PLACEMENT);

static_assert(CONFIG_PLACEMENT + sizeof(DriveConfig) <= CALIBRATION_PLACEMENT ||
              CONFIG_PLACEMENT >= CALIBRATION_PLACEMENT + sizeof(CalibrationData), "Configuration overlaps calibration");
static_assert(CONFIG_PLACEMENT + sizeof(DriveConfig) <= IND_SENSOR_STATE_PLACEMENT ||
              CONFIG_PLACEMENT >= IND_SENSOR_STATE_PLACEMENT + sizeof(InductiveSensor::State), "Configuration overlaps inductive state");
