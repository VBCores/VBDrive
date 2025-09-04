#pragma region Includes
#include "app.h"

#include <memory>

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
VBInverter motor_inverter(&hadc1, &hadc2);

std::shared_ptr<VBDrive> motor;
std::shared_ptr<VBDrive> get_motor() {
    return motor;
}

void create_motor(VBDriveConfig& config_data) {
    // hardware limit
    const float MAX_VOLTAGE = 50.0f;

    // user-defined limit, can be superseded by motor parameters (stall, etc.)
    const float MAX_USER_CURRENT = value_or_default(config_data.max_current, 10.0f);

    motor = std::make_shared<VBDrive>(
        0.00005f,
        // Kalman filter for determining electric angle
        KalmanConfig {
            .expected_a = value_or_default(config_data.filter_a, 0.0f),
            .g1 = value_or_default(config_data.filter_g1, 0.06598266439978051f),
            .g2 = value_or_default(config_data.filter_g1, 37.30880268409287f),
            .g3 = value_or_default(config_data.filter_g1, 8738.342694769584f),
        },
        // Control regulator for dedicated control (velocity and position)
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, 0.0f),
            .ki = value_or_default(config_data.ki, 0.0f),
            .kd = value_or_default(config_data.kd, 0.0f),
            .integral_error_lim = MAX_USER_CURRENT,
            .max_output = MAX_USER_CURRENT,
            .min_output = -MAX_USER_CURRENT,
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = 10.0f,
            .ki = 800.0f,
            .integral_error_lim = MAX_VOLTAGE,
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = 10.0f,
            .ki = 800.0f,
            .integral_error_lim = MAX_VOLTAGE,
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
        &hspi3,
        GpioPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin)
    );
    HAL_Delay(100);
    motor->init();
}

void apply_calibration() {
    CalibrationData calibration_data;
    HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, 0))
    if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
        calibration_data.reset();
        motor->calibrate(calibration_data);
        calibration_data.was_calibrated = true;
        HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, 0))
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

    // separate 20Khz control timer (tim1 is 80Khz, CPU not catching up)
    HAL_TIM_Base_Start_IT(&htim4);

    #ifdef TrackCPUUsage
    micros total_time_us = 0;
    micros idle_time_us = 0;
    micros last_time = micros_64();
    while(true) {
        uint64_t idle_start = micros_64();
        __WFI();  // Idle
        uint64_t idle_end = micros_64();

        idle_time_us += subtract_64(idle_end, idle_start);

        uint64_t now = micros_64();
        total_time_us += subtract_64(now, last_time);
        last_time = now;

        if (total_time_us >= MICROS_S) {
            float usage = 100.0f * (1.0f - ((float)idle_time_us / total_time_us));
            UARTResponseAccumulator responses(&huart2);
            responses.append("FOC CPU usage: %.2f%%\n", usage);
            idle_time_us = 0;
            total_time_us = 0;
        }
    }
    #else
    while(true) {}
    #endif
}

#ifndef NO_CYPHAL
#pragma region Cyphal
TYPE_ALIAS(FloatArray, uavcan_primitive_array_Real32_1_0)
TYPE_ALIAS(EmptyMsg, uavcan_primitive_Empty_1_0)
static constexpr CanardPortID LCM_RX_PORT = 1100;
static constexpr CanardPortID LCM_TX_PORT = 1101;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        FloatArray::Type report_msg = {};
        report_msg.value.count = 8;
        report_msg.value.elements[0] = -motor->get_angle();
        report_msg.value.elements[1] = 0.0f;
        report_msg.value.elements[2] = -motor->get_velocity();
        report_msg.value.elements[3] = 0.0f;
        report_msg.value.elements[4] = -motor->get_torque();
        report_msg.value.elements[5] = 0.0f;
        report_msg.value.elements[6] = 10.0f;
        report_msg.value.elements[7] = 800.0f;
        static CanardTransferID report_msg_transfer_id = 0;
        get_interface()->send_msg<FloatArray>(&report_msg, LCM_RX_PORT, &report_msg_transfer_id);
    })
}


class LCMCommandSub: public AbstractSubscription<FloatArray> {
public:
    LCMCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<FloatArray>(interface, port_id) {};
    void handler(const FloatArray::Type& msg, CanardRxTransfer*) override {
        if (msg.value.count < 7) {
            return;
        }

        motor->set_foc_point(FOCTarget{
            .torque = -msg.value.elements[2],
            .angle = -msg.value.elements[0],
            .velocity = -msg.value.elements[1],
            .angle_kp = -msg.value.elements[4],
            .velocity_kp = -msg.value.elements[5]
        });
    }
};


ReservedObject<LCMCommandSub> lcm_command_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_app_config().get_node_id();

    lcm_command_sub.create(cyphal_interface, LCM_TX_PORT);

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        lcm_command_sub->make_filter(node_id)
    ))
}
#pragma endregion
#endif
