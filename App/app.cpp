#pragma region Includes
#include "app.h"

#include <memory>
#include <type_traits>

#include "tim.h"
#include "i2c.h"
#include "adc.h"

#include "spi.h"
#include "cordic.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <uavcan/si/unit/voltage/Scalar_1_0.h>
#include "uavcan/primitive/array/Real32_1_0.h"
#include "uavcan/primitive/Empty_1_0.h"
#include <voltbro/foc/command_1_0.h>
#include <voltbro/foc/specific_control_1_0.h>
#include <voltbro/foc/state_simple_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
#pragma endregion

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

// correct is_inverted and elec_offset values will be set by apply_calibration
AS5047P motor_encoder(GpioPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin), &hspi1);
InductiveSensor inductive_sensor(
    eeprom,
    IND_SENSOR_STATE_PLACEMENT,
    &hspi3,
    GpioPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin)
);
VBInverter motor_inverter(&hadc1, &hadc2);
alignas(4) static CalibrationData calibration_data;  // avoid stack overflow and misalignment issues
static std::aligned_storage_t<sizeof(VBDrive), alignof(VBDrive)> motor_storage;
static VBDrive* motor = nullptr;
VBDrive* get_motor() {
    return motor;
}

void create_motor(VBDriveConfig& config_data) {
    // hardware limit is 50V, 30A (?)
    constexpr float MAX_VOLTAGE = 50.0f;

    constexpr float DEFAULT_I_KP = 4.0;
    constexpr float DEFAULT_I_KI = 1600.0;

    motor = new (&motor_storage) VBDrive(
        0.000025f,
        // Kalman filter for determining electric angle
        KalmanConfig {
            .expected_a = value_or_default(config_data.filter_a, 0.0f),
            .g1 = 0.015700989410003974f,
            .g2 = 3.925227776360174f,
            .g3 = 387.54711795263574f,
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, DEFAULT_I_KP),
            .ki = value_or_default(config_data.ki, DEFAULT_I_KI),
            .kd = value_or_default(config_data.kd, 0.0f),
            .integral_error_lim = MAX_VOLTAGE,
            .max_output = MAX_VOLTAGE,
            .min_output = -MAX_VOLTAGE,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, DEFAULT_I_KP),
            .ki = value_or_default(config_data.ki, DEFAULT_I_KI),
            .kd = value_or_default(config_data.kd, 0.0f),
            .integral_error_lim = MAX_VOLTAGE,
            .max_output = MAX_VOLTAGE,
            .min_output = -MAX_VOLTAGE,
        },
        // User-defined limits
        DriveLimits {
            .user_current_limit = value_or_default(config_data.max_current, NAN),
            .user_torque_limit = value_or_default(config_data.max_torque, NAN),
            .user_speed_limit = value_or_default(config_data.max_speed, NAN),
            .user_position_lower_limit = value_or_default(config_data.min_angle, NAN),
            .user_position_upper_limit = value_or_default(config_data.max_angle, NAN),
        },
        // Built-in constant parameters
        DriveInfo {
            .torque_const = value_or_default(config_data.torque_const, 1.0f),
            .max_current = 30.0,
            .max_torque = 100.0f,
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.2f,
            .en_pin = GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin),
            .common = {
                .ppairs = 14,
                .gear_ratio = value_or_default(
                    config_data.gear_ratio,
                    static_cast<uint8_t>(36),
                    static_cast<uint8_t>(0)
            ),
                .user_angle_offset = value_or_default(config_data.angle_offset, 0.0f)
            }
        },
        &htim1,
        motor_encoder,
        motor_inverter,
        inductive_sensor
    );
    HAL_Delay(100);
    motor->init();
}

void apply_calibration() {
    if (calibration_data.type_id == 0) {  // uninitialized, try to read from EEPROM
        HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    }
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        calibration_data.reset();
        // NOTE: see app.h lines 51-53 for details on cyphal_queue_buffer_shared
        motor->calibrate(calibration_data, cyphal_queue_buffer_shared, SHARED_BUFFER_SIZE);
        calibration_data.was_calibrated = true;
        HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    }
    motor->apply_calibration(calibration_data);
}

[[noreturn]] void app() {
#pragma region StartupConfiguration
    start_timers();
    eeprom.wait_until_available();
    auto& app_config = get_app_config();
    app_config.init();
    auto& config_data = app_config.get_config();

    if (!app_config.is_app_running()) {
        // Hold here until configured and rebooted (in interrupt handler in config.cpp)
        while (true) {}
    }

    setup_cordic();
    create_motor(config_data);
    motor->start();
    apply_calibration();
    motor->set_voltage_point(0.0f);

    start_cyphal();
    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Lock heap
    // NOTE: heap is used only by control block of shared_ptr,
    //       so in order to keep same API with Linux, we allow tiny heap
    global_allocation_lock = true;
#pragma endregion

    HAL_TIM_Base_Start_IT(&htim4);

    while(true) {
        cyphal_loop();
    }
}

#ifndef NO_CYPHAL
#pragma region Cyphal
#ifdef LCM
#include "lcm.h"
#else
TYPE_ALIAS(FOCCommand, voltbro_foc_command_1_0)
TYPE_ALIAS(FOCState, voltbro_foc_state_simple_1_0)
TYPE_ALIAS(SpecificControl, voltbro_foc_specific_control_1_0)

static constexpr CanardPortID FOC_COMMAND_PORT = 2107;
static constexpr CanardPortID FOC_STATE_PORT = 3811;
static constexpr CanardPortID SPECIFIC_CONTROL_PORT = 3407;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        FOCState::Type state_msg = {};

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
        get_interface()->send_msg<FOCState>(&state_msg, FOC_STATE_PORT, &state_transfer_id);
    })
}

class FOCCommandSub: public AbstractSubscription<FOCCommand> {
public:
    FOCCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<FOCCommand>(interface, port_id) {};
    void handler(const FOCCommand::Type& msg, CanardRxTransfer* _) override {
        motor->set_foc_point(FOCTarget {
            .torque = msg._torque.newton_meter,
            .angle = msg.angle.radian,
            .velocity = msg.velocity.radian_per_second,
            .angle_kp = msg.angle_kp.value,
            .velocity_kp = msg.velocity_kp.value
        });
        motor->set_current_regulator_params(msg.I_kp.value, msg.I_ki.value);
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
// TODO: Specific control
//ReservedObject<SpecificControlSub> specific_control_sub;
ReservedObject<FOCCommandSub> foc_command_sub;
#endif

void setup_subscriptions() {
    auto cyphal_interface = get_interface();

#ifdef LCM
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE
    );
    lcm_command_sub.create(cyphal_interface);
#else
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    const auto node_id = get_app_config().get_node_id();
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    motor->set_state(value);

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    v_out.bit.value.bitpacked[0] = motor->is_on();
                    v_out.bit.value.count = 1;
                }
            }
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

    // TODO: specific control
    //specific_control_sub.create(cyphal_interface, SPECIFIC_CONTROL_PORT + node_id);
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

    // TODO: Specific control
    /*
    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        specific_control_sub->make_filter(node_id)
    ))
    */
#endif
}
#pragma endregion
#endif
