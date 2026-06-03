//#pragma region Includes
#include "app.h"

#include <memory>
#include <type_traits>

#include "tim.h"
#include "i2c.h"
#include "adc.h"

#include "spi.h"
#include "cordic.h"

#include <voltbro/devices/stspin32g4.hpp>
#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/node/registers_utils.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.hpp>
#include <uavcan/primitive/Empty_1_0.hpp>
#include <uavcan/primitive/array/Real32_1_0.hpp>
#include <uavcan/primitive/scalar/Real32_1_0.hpp>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.hpp>
#include <uavcan/si/unit/angle/Scalar_1_0.hpp>
#include <uavcan/si/unit/torque/Scalar_1_0.hpp>
#include <uavcan/si/unit/voltage/Scalar_1_0.hpp>
#include <voltbro/foc/command_1_0.hpp>
#include <voltbro/foc/specific_control_1_0.hpp>
#include <voltbro/foc/state_simple_1_0.hpp>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
//#pragma endregion

static constexpr CanardNodeID UNCONFIGURED_SETUP_NODE_ID = 126;

//#pragma region ExternConfiguration
#define NANOPRINTF_IMPLEMENTATION
#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS   1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS       1 // float
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS       1 // 'l' (long), 'll' (long long)
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS       1 // 'hh' (char), 'h' (short)
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS      0 // %b (binary)
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS   0 // %n
#include "nanoprintf.h"
extern "C" {
    // During setup we need tiny heap for shared_ptr of CyphalInterface for API compatibility reasons
    bool global_allocation_lock = false;
}
//#pragma endregion

#ifdef FOC_PROFILE
extern uint32_t __StackLimit;
extern uint32_t __StackTop;
constexpr uint32_t STACK_CANARY = 0xDEADBEEF;
volatile static size_t max_stack_usage = 0;

void mark_stack() {
    uint32_t current_sp;
    __asm__ volatile ("mov %0, sp" : "=r" (current_sp));

    volatile uint32_t *p = static_cast<volatile uint32_t*>(&__StackLimit);
    volatile uint32_t *sp = reinterpret_cast<volatile uint32_t*>(current_sp);

    while (p < sp) {
        *p = STACK_CANARY;
        p++;
    }
}

void measure_stack_usage() {
    volatile uint32_t *p = static_cast<volatile uint32_t*>(&__StackLimit);
    volatile uint32_t *top = static_cast<volatile uint32_t*>(&__StackTop);

    while (p < top && *p == STACK_CANARY) {
        p++;
    }

    size_t unused = static_cast<size_t>(p - &__StackLimit);
    size_t total = static_cast<size_t>(&__StackTop - &__StackLimit);
    size_t used = total - unused;

    if (used > max_stack_usage) {
        max_stack_usage = used;
    }
}
#endif

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

static int8_t config_angle_direction(const VBDriveConfig& config_data) {
    if (config_data.max_voltage == -1.0f) {
        return -1;
    }
    return 1;
}

void create_motor(VBDriveConfig& config_data) {
    motor = new (&motor_storage) VBDrive(
        0.000025f,
        // Kalman filter for determining electric angle
        FiltersConfig {
            .expected_a = value_or_default(config_data.filter_a, VBDriveDefaults::FILTER_A),
            .g1 = value_or_default(config_data.filter_g1, VBDriveDefaults::FILTER_G1),
            .g2 = value_or_default(config_data.filter_g2, VBDriveDefaults::FILTER_G2),
            .g3 = value_or_default(config_data.filter_g3, VBDriveDefaults::FILTER_G3),
            .I_lpf_coefficient = value_or_default(config_data.I_lpf_coefficient, VBDriveDefaults::I_LPF)
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, VBDriveDefaults::PID_KP),
            .ki = value_or_default(config_data.ki, VBDriveDefaults::PID_KI),
            .kd = value_or_default(config_data.kd, VBDriveDefaults::PID_KD),
            .integral_error_lim = VBDriveDefaults::MAX_VOLTAGE,
            .max_output = VBDriveDefaults::MAX_VOLTAGE,
            .min_output = -VBDriveDefaults::MAX_VOLTAGE,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, VBDriveDefaults::PID_KP),
            .ki = value_or_default(config_data.ki, VBDriveDefaults::PID_KI),
            .kd = value_or_default(config_data.kd, VBDriveDefaults::PID_KD),
            .integral_error_lim = VBDriveDefaults::MAX_VOLTAGE,
            .max_output = VBDriveDefaults::MAX_VOLTAGE,
            .min_output = -VBDriveDefaults::MAX_VOLTAGE,
        },
        // User-defined limits
        DriveLimits {
            .user_current_limit = value_or_default(config_data.max_current, NAN),
            .user_torque_limit = value_or_default(config_data.max_torque, NAN),
            .user_speed_limit = value_or_default(config_data.max_speed, NAN),
            .user_position_lower_limit = value_or_default(config_data.min_angle, NAN),
            .user_position_upper_limit = value_or_default(config_data.max_angle, NAN),
            .user_angle_offset = value_or_default(config_data.angle_offset, VBDriveDefaults::ANGLE_OFFSET),
            .user_angle_direction = config_angle_direction(config_data)
        },
        // Built-in constant parameters
        DriveInfo {
            .torque_const = value_or_default(config_data.torque_const, VBDriveDefaults::TORQUE_CONST),
            .max_current = 30.0,
            .max_torque = 30.0f,
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.2f,
            .en_pin = GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin),
            .common = {
                .ppairs = 14,
                .gear_ratio = value_or_default(
                    config_data.gear_ratio,
                    VBDriveDefaults::GEAR_RATIO,
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

bool do_calibrate() {
    // Stop all control
    motor->set_foc_point(FOCTarget{0});
    auto& app_manager = get_app_manager();
    app_manager.set_state(CommandState::NOT_CALIBRATED);

    calibration_data.reset();
    // NOTE: see app.h lines 20-21 for details on cyphal_queue_buffer_shared
    motor->calibrate(calibration_data, cyphal_queue_buffer_shared, SHARED_BUFFER_SIZE);
    calibration_data.was_calibrated = true;
    HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    motor->apply_calibration(calibration_data);

    app_manager.set_state(CommandState::RUNNING);
    return true;
}

void apply_calibration() {
    if (calibration_data.type_id == 0) {  // uninitialized, try to read from EEPROM
        HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    }
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        auto& app_manager = get_app_manager();
        char warning_message[] = "Motor is not calibrated! Movement forbidden\n\r\0";
        app_manager.send_message_blocking(warning_message);
        app_manager.set_state(CommandState::NOT_CALIBRATED);
        return;
    }
    motor->apply_calibration(calibration_data);
}

static void persist_pending_config_if_needed();
static void reboot_to_bootloader_if_requested();

void app() {
#ifdef FOC_PROFILE
    mark_stack();
#endif
//#pragma region StartupConfiguration
    start_timers();
    eeprom.wait_until_available();
    auto& app_manager = get_app_manager();
    app_manager.init();
    start_uart_recv_it();
    auto& config_data = app_manager.get_config();
    if (!app_manager.is_app_running() && config_data.are_required_params_set()) {
        config_data.was_configured = true;
        app_manager.set_state(CommandState::RUNNING);
    }

    if (!app_manager.is_app_running()) {
        // Bring up Cyphal even with blank EEPROM so config can be restored over CAN.
        if (config_data.node_id == 0) {
            config_data.node_id = UNCONFIGURED_SETUP_NODE_ID;
        }
        start_cyphal();
        set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);
        while (true) {
            cyphal_loop();
            persist_pending_config_if_needed();
            reboot_to_bootloader_if_requested();
            if (app_manager.is_app_running()) {
                HAL_NVIC_SystemReset();
            }
        }
    }

    setup_cordic();
    create_motor(config_data);
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
//#pragma endregion

    HAL_TIM_Base_Start_IT(&htim4);

    #ifdef FOC_PROFILE
    static millis stack_measurement_time = 0;
    #endif
    static millis logging_time = 0;

    while(true) {
        if (app_manager.is_app_running()) {
            cyphal_loop();
            persist_pending_config_if_needed();
            reboot_to_bootloader_if_requested();
        }

        millis current_time = millis_32();
        #ifdef FOC_PROFILE
        EACH_N(current_time, stack_measurement_time, 100, {
            measure_stack_usage();
        })
        #endif

        EACH_N(current_time, logging_time, 100, {
            if (app_manager.is_logging()) {
                app_manager.send_message_blocking(
                    "rotor: %6u shaft :%6u angle: %6.2f velocity: %6.2f\r\n",
                    motor->get_rotor_encoder_value(),
                    motor->get_shaft_encoder_value(),
                    motor->get_angle(),
                    motor->get_velocity()
                );
            }
        })
    }
}

#ifndef NO_CYPHAL
//#pragma region Cyphal
using FOCCommand = voltbro_foc_command_1_0;
using FOCState = voltbro_foc_state_simple_1_0;
using SpecificControl = voltbro_foc_specific_control_1_0;

static constexpr CanardPortID FOC_COMMAND_PORT = 2107;
static constexpr CanardPortID FOC_STATE_PORT = 3811;
static constexpr CanardPortID SPECIFIC_CONTROL_PORT = 3407;

static uint32_t invalid_commands_counter = 0;
static bool config_save_pending = false;
static bool bootloader_reboot_pending = false;

using ConfigFloatSetter = void (*)(VBDriveConfig&, float);
using ConfigFloatGetter = float (*)(const VBDriveConfig&);
using ConfigU32Setter = bool (*)(VBDriveConfig&, uint32_t);
using ConfigU32Getter = uint32_t (*)(const VBDriveConfig&);

static bool request_config_save(VBDriveConfig& config) {
    config.was_configured = config.are_required_params_set();
    config_save_pending = true;
    return true;
}

static void persist_pending_config_if_needed() {
    if (!config_save_pending) {
        return;
    }
    config_save_pending = false;
    auto& config = get_app_manager().get_config();
    config.was_configured = config.are_required_params_set();
    HAL_IMPORTANT(get_eeprom().write<VBDriveConfig>(&config, CONFIG_PLACEMENT))
}

static void reboot_to_bootloader_if_requested() {
    if (!bootloader_reboot_pending) {
        return;
    }
    bootloader_reboot_pending = false;
    reboot_to_bootloader();
}

static bool update_persistent_float_register(
    float DriveLimits::* limits_field,
    ConfigFloatSetter config_setter,
    float value
) {
    DriveLimits limits = motor->get_limits();
    limits.*limits_field = value;
    if (!motor->set_limits(limits)) {
        return false;
    }

    auto& config = get_app_manager().get_config();
    config_setter(config, value);
    return request_config_save(config);
}

static bool update_persistent_config_float_register(ConfigFloatSetter config_setter, float value) {
    auto& config = get_app_manager().get_config();
    config_setter(config, value);
    return request_config_save(config);
}

static bool update_persistent_config_u32_register(ConfigU32Setter config_setter, uint32_t value) {
    auto& config = get_app_manager().get_config();
    if (!config_setter(config, value)) {
        return false;
    }
    return request_config_save(config);
}

static bool update_persistent_direction_register(int32_t value) {
    if (value != -1 && value != 1) {
        return false;
    }

    DriveLimits limits = motor->get_limits();
    limits.user_angle_direction = static_cast<int8_t>(value);
    if (!motor->set_limits(limits)) {
        return false;
    }

    auto& config = get_app_manager().get_config();
    // Keep EEPROM layout stable: max_voltage is not used by VBDrive runtime.
    config.max_voltage = static_cast<float>(value);
    return request_config_save(config);
}


void in_loop_reporting(millis current_t) {
    if (motor == nullptr) {
        return;
    }

    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        FOCState state_msg = {};

        state_msg.timestamp.microsecond = system_time();

        state_msg.angle.radian = motor->get_angle();
        state_msg.velocity.radian_per_second = motor->get_velocity();
        state_msg._torque.newton_meter = motor->get_torque();

        state_msg.current.ampere = motor->get_working_current();
        state_msg.bus_voltage.volt = motor_inverter.get_busV();

        constexpr float KELVIN_OFFSET = 273.15f;
        motor_inverter.update_temperature();
        state_msg.mcu_temp.kelvin = motor_inverter.get_mcu_temperature() + KELVIN_OFFSET;
        state_msg.stator_temp.kelvin = motor_inverter.get_stator_temperature() + KELVIN_OFFSET;

        state_msg.has_fault.value = false; // TODO: fault check

        static CanardTransferID state_transfer_id = 0;
        get_interface()->send_msg(&state_msg, FOC_STATE_PORT, &state_transfer_id);
    })
}

class FOCCommandSub: public AbstractSubscription<FOCCommand> {
public:
    FOCCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<FOCCommand>(interface, port_id) {};
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    // NOTE: transfer parameter required by the interface, but not used in this implementation
    void handler(const FOCCommand& msg, CanardRxTransfer* _) override {
    #pragma GCC diagnostic pop
        bool is_valid = motor->set_foc_point(FOCTarget {
            .torque = msg._torque.newton_meter,
            .angle = msg.angle.radian,
            .velocity = msg.velocity.radian_per_second,
            .angle_kp = msg.angle_kp.value,
            .velocity_kp = msg.velocity_kp.value
        });
        if (!is_valid) {
            invalid_commands_counter += 1;
        }
        motor->set_current_regulator_params(msg.I_kp.value, msg.I_ki.value);
    }
};

class SpecificControlSub: public AbstractSubscription<SpecificControl> {
public:
    SpecificControlSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<SpecificControl>(interface, port_id) {};
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    // NOTE: transfer parameter required by the interface, but not used in this implementation
    void handler(const SpecificControl& msg, CanardRxTransfer* _) override {
    #pragma GCC diagnostic pop
        bool is_valid = false;
        switch (msg.set_point_type){
            case voltbro_foc_specific_control_1_0_VELOCITY:
                is_valid = motor->set_velocity_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_TORQUE:
                is_valid = motor->set_torque_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_POSITION:
                is_valid = motor->set_angle_point(msg.set_point_value);
                break;
            case voltbro_foc_specific_control_1_0_VOLTAGE:
                is_valid = motor->set_voltage_point(msg.set_point_value);
                break;
            default:
                break;
        }
        if (!is_valid) {
            invalid_commands_counter += 1;
        }
    }
};

// NOTE: underlying CanardRxSubscriptions are HUGE - 552 bytes each. C++ wrapper size is negligible in comparison
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<24>> registers_handler;
ReservedObject<FOCCommandSub> foc_command_sub;
ReservedObject<SpecificControlSub> specific_control_sub;

static bool parse_bool_register_value_robust(const uavcan_register_Value_1_0& value, bool& parsed) {
    if (parse_register_bit(value, parsed)) {
        return true;
    }
    if (value.integer32.value.count > 0) {
        parsed = value.integer32.value.elements[0] != 0;
        return true;
    }
    if (value.natural32.value.count > 0) {
        parsed = value.natural32.value.elements[0] != 0U;
        return true;
    }
    if (value.real32.value.count > 0) {
        parsed = value.real32.value.elements[0] != 0.0f;
        return true;
    }
    return false;
}

void setup_subscriptions() {
    auto cyphal_interface = get_interface();

    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    const auto node_id = get_app_manager().get_node_id();
    auto make_persistent_float_register = [](
        const char* name,
        float DriveLimits::* limits_field,
        ConfigFloatSetter config_setter
    ) -> RegisterDefinition {
        return {
            name,
            [limits_field, config_setter](
                const uavcan_register_Value_1_0& v_in,
                uavcan_register_Value_1_0& v_out,
                RegisterAccessResponse& response
            ) {
                if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                    float value = 0.0f;
                    if (parse_register_real32(v_in, value)) {
                        update_persistent_float_register(limits_field, config_setter, value);
                    }
                }

                response.persistent = true;
                response._mutable = true;
                fill_register_real32(v_out, motor->get_limits().*limits_field);
            }
        };
    };
    auto make_config_float_register = [](
        const char* name,
        ConfigFloatGetter config_getter,
        ConfigFloatSetter config_setter
    ) -> RegisterDefinition {
        return {
            name,
            [config_getter, config_setter](
                const uavcan_register_Value_1_0& v_in,
                uavcan_register_Value_1_0& v_out,
                RegisterAccessResponse::Type& response
            ) {
                if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                    float value = 0.0f;
                    if (parse_register_real32(v_in, value)) {
                        update_persistent_config_float_register(config_setter, value);
                    }
                }

                response.persistent = true;
                response._mutable = true;
                fill_register_real32(v_out, config_getter(get_app_manager().get_config()));
            }
        };
    };
    auto make_config_u32_register = [](
        const char* name,
        ConfigU32Getter config_getter,
        ConfigU32Setter config_setter
    ) -> RegisterDefinition {
        return {
            name,
            [config_getter, config_setter](
                const uavcan_register_Value_1_0& v_in,
                uavcan_register_Value_1_0& v_out,
                RegisterAccessResponse::Type& response
            ) {
                if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                    uint32_t value = 0;
                    int32_t signed_value = 0;
                    if (parse_register_natural32(v_in, value) ||
                        (parse_register_integer32(v_in, signed_value) && signed_value >= 0)) {
                        if (v_in._tag_ == REGISTER_INTEGER32_TAG) {
                            value = static_cast<uint32_t>(signed_value);
                        }
                        update_persistent_config_u32_register(config_setter, value);
                    }
                }

                response.persistent = true;
                response._mutable = true;
                fill_register_natural32(v_out, config_getter(get_app_manager().get_config()));
            }
        };
    };

    registers_handler.create(
        std::array<RegisterDefinition, 24>{{
            {
                "state.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse& response
                ){
                    if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                        bool value = false;
                        if (parse_bool_register_value_robust(v_in, value)) {
                            motor->set_state(value);
                        }
                    }

                    response.persistent = false;
                    response._mutable = true;
                    fill_register_bit(v_out, motor->is_on());
                }
            },
            {
                "state.errors",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse& response
                ){
                    (void) v_in;
                    response.persistent = false;
                    response._mutable = false;
                    fill_register_natural32(v_out, invalid_commands_counter);
                }
            },
            {
                "command.bootloader",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                        bool value = false;
                        if (parse_bool_register_value_robust(v_in, value) && value) {
                            bootloader_reboot_pending = true;
                        }
                    }

                    response.persistent = false;
                    response._mutable = true;
                    fill_register_bit(v_out, bootloader_reboot_pending);
                }
            },
            make_persistent_float_register(
                "limit.current",
                &DriveLimits::user_current_limit,
                [](VBDriveConfig& config, float value) { config.max_current = value; }
            ),
            make_persistent_float_register(
                "limit.torque",
                &DriveLimits::user_torque_limit,
                [](VBDriveConfig& config, float value) { config.max_torque = value; }
            ),
            make_persistent_float_register(
                "limit.speed",
                &DriveLimits::user_speed_limit,
                [](VBDriveConfig& config, float value) { config.max_speed = value; }
            ),
            make_persistent_float_register(
                "limit.min_angle",
                &DriveLimits::user_position_lower_limit,
                [](VBDriveConfig& config, float value) { config.min_angle = value; }
            ),
            make_persistent_float_register(
                "limit.max_angle",
                &DriveLimits::user_position_upper_limit,
                [](VBDriveConfig& config, float value) { config.max_angle = value; }
            ),
            make_persistent_float_register(
                "angle.offset",
                &DriveLimits::user_angle_offset,
                [](VBDriveConfig& config, float value) { config.angle_offset = value; }
            ),
            {
                "angle.direction",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    if (v_in._tag_ != REGISTER_EMPTY_TAG) {
                        int32_t value = 0;
                        if (parse_register_integer32(v_in, value)) {
                            update_persistent_direction_register(value);
                        }
                    }

                    response.persistent = true;
                    response._mutable = true;
                    fill_register_integer32(v_out, motor->get_limits().user_angle_direction);
                }
            },
            make_config_u32_register(
                "node.id",
                [](const VBDriveConfig& config) { return static_cast<uint32_t>(config.node_id); },
                [](VBDriveConfig& config, uint32_t value) {
                    if (value == 0 || value > 127) {
                        return false;
                    }
                    config.node_id = static_cast<CanardNodeID>(value);
                    return true;
                }
            ),
            make_config_u32_register(
                "config.gear",
                [](const VBDriveConfig& config) { return static_cast<uint32_t>(value_or_default(config.gear_ratio, VBDriveDefaults::GEAR_RATIO, static_cast<uint8_t>(0))); },
                [](VBDriveConfig& config, uint32_t value) {
                    if (value == 0 || value > UINT8_MAX) {
                        return false;
                    }
                    config.gear_ratio = static_cast<uint8_t>(value);
                    return true;
                }
            ),
            make_config_u32_register(
                "config.nominal_baud",
                [](const VBDriveConfig& config) { return static_cast<uint32_t>(config.fdcan_nominal_baud); },
                [](VBDriveConfig& config, uint32_t value) {
                    if (value > static_cast<uint32_t>(FDCANNominalBaud::KHz1000)) {
                        return false;
                    }
                    config.fdcan_nominal_baud = static_cast<FDCANNominalBaud>(value);
                    return true;
                }
            ),
            make_config_u32_register(
                "config.data_baud",
                [](const VBDriveConfig& config) { return static_cast<uint32_t>(config.fdcan_data_baud); },
                [](VBDriveConfig& config, uint32_t value) {
                    if (value > static_cast<uint32_t>(FDCANDataBaud::KHz8000)) {
                        return false;
                    }
                    config.fdcan_data_baud = static_cast<FDCANDataBaud>(value);
                    return true;
                }
            ),
            make_config_u32_register(
                "config.angle_encoder",
                [](const VBDriveConfig& config) { return static_cast<uint32_t>(to_underlying(config.angle_encoder)); },
                [](VBDriveConfig& config, uint32_t value) {
                    if (value > static_cast<uint32_t>(to_underlying(AngleEncoderType::SHAFT))) {
                        return false;
                    }
                    config.angle_encoder = static_cast<AngleEncoderType>(value);
                    return true;
                }
            ),
            make_config_float_register(
                "motor.torque_constant",
                [](const VBDriveConfig& config) { return value_or_default(config.torque_const, VBDriveDefaults::TORQUE_CONST); },
                [](VBDriveConfig& config, float value) { config.torque_const = value; }
            ),
            make_config_float_register(
                "foc.kp",
                [](const VBDriveConfig& config) { return value_or_default(config.kp, VBDriveDefaults::PID_KP); },
                [](VBDriveConfig& config, float value) { config.kp = value; }
            ),
            make_config_float_register(
                "foc.ki",
                [](const VBDriveConfig& config) { return value_or_default(config.ki, VBDriveDefaults::PID_KI); },
                [](VBDriveConfig& config, float value) { config.ki = value; }
            ),
            make_config_float_register(
                "foc.kd",
                [](const VBDriveConfig& config) { return value_or_default(config.kd, VBDriveDefaults::PID_KD); },
                [](VBDriveConfig& config, float value) { config.kd = value; }
            ),
            make_config_float_register(
                "filter.a",
                [](const VBDriveConfig& config) { return value_or_default(config.filter_a, VBDriveDefaults::FILTER_A); },
                [](VBDriveConfig& config, float value) { config.filter_a = value; }
            ),
            make_config_float_register(
                "filter.g1",
                [](const VBDriveConfig& config) { return value_or_default(config.filter_g1, VBDriveDefaults::FILTER_G1); },
                [](VBDriveConfig& config, float value) { config.filter_g1 = value; }
            ),
            make_config_float_register(
                "filter.g2",
                [](const VBDriveConfig& config) { return value_or_default(config.filter_g2, VBDriveDefaults::FILTER_G2); },
                [](VBDriveConfig& config, float value) { config.filter_g2 = value; }
            ),
            make_config_float_register(
                "filter.g3",
                [](const VBDriveConfig& config) { return value_or_default(config.filter_g3, VBDriveDefaults::FILTER_G3); },
                [](VBDriveConfig& config, float value) { config.filter_g3 = value; }
            ),
            make_config_float_register(
                "filter.i_lpf",
                [](const VBDriveConfig& config) { return value_or_default(config.I_lpf_coefficient, VBDriveDefaults::I_LPF); },
                [](VBDriveConfig& config, float value) { config.I_lpf_coefficient = value; }
            )
        }},
        cyphal_interface
    );

    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.vbdrive",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );

    specific_control_sub.create(cyphal_interface, SPECIFIC_CONTROL_PORT + node_id);
    foc_command_sub.create(cyphal_interface, FOC_COMMAND_PORT + node_id);

    HAL_IMPORTANT(apply_filter(
        0,
        &hfdcan1,
        foc_command_sub->make_filter(node_id)
    ))

    HAL_IMPORTANT(apply_filter(
        1,
        &hfdcan1,
        registers_handler->make_filter(node_id)
    ))

    HAL_IMPORTANT(apply_filter(
        2,
        &hfdcan1,
        node_info_reader->make_filter(node_id)
    ))

    HAL_IMPORTANT(apply_filter(
        3,
        &hfdcan1,
        specific_control_sub->make_filter(node_id)
    ))
}
//#pragma endregion
#endif
