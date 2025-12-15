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

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/utils.hpp>
#pragma endregion

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
// Static storage to avoid heap usage in the hot path.
static std::aligned_storage_t<sizeof(VBDrive), alignof(VBDrive)> motor_storage;
static VBDrive* motor = nullptr;
VBDrive* get_motor() {
    return motor;
}

void create_motor(VBDriveConfig& config_data) {
    // hardware limit
    constexpr float MAX_VOLTAGE = 50.0f;
    // user-defined limit, can be superseded by motor parameters (stall, etc.)
    const float MAX_USER_CURRENT = value_or_default(config_data.max_current, 10.0f);
    constexpr float DEFAULT_I_KP = 3.0;
    constexpr float DEFAULT_I_KI = 3.0;

    motor = new (&motor_storage) VBDrive(
        0.000025f,
        // Kalman filter for determining electric angle
        KalmanConfig {
            .expected_a = value_or_default(config_data.filter_a, 0.0f),
            .g1 = 0.031f,
            .g2 = 7.788f,
            .g3 = 768.99f,
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, DEFAULT_I_KP),
            .ki = value_or_default(config_data.ki, DEFAULT_I_KI),
            .kd = value_or_default(config_data.kd, 0.0f),
            .integral_error_lim = MAX_VOLTAGE,
            .max_output = MAX_USER_CURRENT,
            .min_output = -MAX_USER_CURRENT,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, DEFAULT_I_KP),
            .ki = value_or_default(config_data.ki, DEFAULT_I_KI),
            .kd = value_or_default(config_data.kd, 0.0f),
            .integral_error_lim = MAX_VOLTAGE,
            .max_output = MAX_USER_CURRENT,
            .min_output = -MAX_USER_CURRENT,
        },
        // User-defined limits
        DriveLimits {
            .user_current_limit = MAX_USER_CURRENT,
            .user_torque_limit = value_or_default(config_data.max_torque, NAN),
            .user_speed_limit = value_or_default(config_data.max_speed, NAN),
            .user_position_lower_limit = value_or_default(config_data.min_angle, NAN),
            .user_position_upper_limit = value_or_default(config_data.max_angle, NAN),
        },
        // Built-in constant parameters
        DriveInfo {
            .torque_const = value_or_default(config_data.torque_const, 1.0f),
            .max_current = 10.0f,
            .max_torque = 100.0f,
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.5f,
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
    CalibrationData calibration_data;
    HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, CALIBRATION_PLACEMENT))
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        calibration_data.reset();
        motor->calibrate(calibration_data);
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
    setup_cordic();
    start_cyphal();
    #pragma endregion

    create_motor(config_data);

    if (!app_config.is_app_running()) {
        // Hold here until configured and rebooted (in interrupt handler in config.cpp)
        while (true) {}
    }

    motor->start();
    apply_calibration();
    motor->set_voltage_point(0.0f);

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // separate 20KHz control timer (tim1 is 80KHz, CPU not catching up)
    HAL_TIM_Base_Start_IT(&htim4);

    #ifdef FOC_PROFILE
    motor->set_foc_point(FOCTarget{
        .torque = 0,
        .angle = 0,
        .velocity = 2,
        .angle_kp = 0,
        .velocity_kp = 1.5
    });
    #endif

    while(true) {
        cyphal_loop();
    }
}

#ifndef NO_CYPHAL
#pragma region Cyphal
#ifdef LCM
TYPE_ALIAS(FloatArray, uavcan_primitive_array_Real32_1_0)
TYPE_ALIAS(EmptyMsg, uavcan_primitive_Empty_1_0)
static constexpr CanardPortID LCM_RX_PORT = 1100;
static constexpr CanardPortID LCM_TX_PORT = 1101;
static constexpr CanardPortID LCM_REQUEST_PORT = 900;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        static CanardTransferID report_msg_transfer_id = 0;
        FloatArray::Type report_msg = {};

        report_msg.value.count = 7;
        report_msg.value.elements[0] = motor->get_angle();
        report_msg.value.elements[1] = motor->get_velocity();
        report_msg.value.elements[2] = motor->get_torque();
        report_msg.value.elements[3] = 0.0f;
        report_msg.value.elements[4] = 0.0;
        report_msg.value.elements[5] = 0.0f;
        report_msg.value.elements[6] = 0.0f;

        get_interface()->send_msg<FloatArray>(&report_msg, LCM_TX_PORT, &report_msg_transfer_id);
    })
}


class LCMCommandSub: public AbstractSubscription<FloatArray> {
public:
    LCMCommandSub(InterfacePtr interface): AbstractSubscription<FloatArray>(interface, LCM_RX_PORT) {};
    void handler(const FloatArray::Type& msg, CanardRxTransfer*) override {
        motor->set_foc_point(FOCTarget{
            .torque = msg.value.elements[4],
            .angle = msg.value.elements[0],
            .velocity = msg.value.elements[2],
            .angle_kp = msg.value.elements[1],
            .velocity_kp = msg.value.elements[3]
        });
        motor->set_current_regulator_params(PIDConfig{
            .multiplier = 1.0f,
            .kp = msg.value.elements[6],
            .ki = msg.value.elements[7],
            .kd = 0.0f,
            .integral_error_lim = 20.0f,
            .max_output = 20.0f,
            .min_output = -20.0f,
        });
    }
};


/*
class LCMRequestSub: public AbstractSubscription<EmptyMsg> {
protected:
    CanardTransferID report_msg_transfer_id = 0;
public:
    LCMRequestSub(InterfacePtr interface)
        : AbstractSubscription<EmptyMsg>(interface, LCM_REQUEST_PORT, CanardTransferKindMessage)
        {};
    void handler(const EmptyMsg::Type& msg, CanardRxTransfer*) override {
        FloatArray::Type report_msg = {};
        report_msg.value.count = 7;
        report_msg.value.elements[0] = motor->get_angle();
        report_msg.value.elements[1] = motor->get_velocity();
        report_msg.value.elements[2] = motor->get_torque();
        report_msg.value.elements[3] = 0.0f;
        report_msg.value.elements[4] = 0.0;
        report_msg.value.elements[5] = 0.0f;
        report_msg.value.elements[6] = 0.0f;
        get_interface()->send_msg<FloatArray>(&report_msg, LCM_TX_PORT, &report_msg_transfer_id);
    }
};
*/


ReservedObject<LCMCommandSub> lcm_command_sub;
//ReservedObject<LCMRequestSub> lcm_request_sub;


void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE
    );

    auto cyphal_interface = get_interface();

    lcm_command_sub.create(cyphal_interface);
    //lcm_request_sub.create(cyphal_interface);
}
#endif
#pragma endregion
#endif
