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
// TODO shaft_encoder(GpioPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin), &hspi3);

std::shared_ptr<VBDrive> motor;
std::shared_ptr<VBDrive> get_motor() {
    return motor;
}
CalibrationData calibration_data;

[[noreturn]] void app() {
    start_timers();

    eeprom.wait_until_available();
    auto& app_config = get_app_config();
    app_config.init();
    auto& config_data = app_config.get_config();

    setup_cordic();
    start_cyphal();

    motor = std::make_shared<VBDrive>(
        0.00005f,
        KalmanConfig {
            .expected_a = value_or_default(config_data.filter_a, 0),
            .g1 = value_or_default(config_data.filter_g1, 0.007555786034316281f),
            .g2 = value_or_default(config_data.filter_g1, 0.47474348575882175f),
            .g3 = value_or_default(config_data.filter_g1, 12.355571499256257f),
        },
        // Main control regulator (velocity and position)
        PIDConfig {
            .multiplier = 1.0f,
            .kp = value_or_default(config_data.kp, 0.0f),
            .ki = value_or_default(config_data.ki, 0.0f),
            .kd = value_or_default(config_data.kd, 0.0f),
        },
        // Q Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = 10.0f,
            .ki = 800.0f
        },
        // D Regulator
        PIDConfig {
            .multiplier = 1.0f,
            .kp = 10.0f,
            .ki = 800.0f,
        },
        DriveLimits {
            .user_current_limit = value_or_default(config_data.max_current, 10.0f),
            .user_torque_limit = value_or_default(config_data.max_torque, NAN),
            .user_speed_limit = value_or_default(config_data.max_speed, NAN),
            .user_position_lower_limit = value_or_default(config_data.min_angle, NAN),
            .user_position_upper_limit = value_or_default(config_data.max_angle, NAN),
        },
        DriveInfo {
            .torque_const = value_or_default(config_data.torque_const, 0.42f),
            .max_current = 10.0f,
            .max_torque = 10.0f,
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.5f,
            .en_pin = GpioPin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin),
            .common = {
                .ppairs = 14,
                .gear_ratio = 64,
                .user_angle_offset = value_or_default(config_data.angle_offset, 0.0f)
            }
        },
        &htim1,
        motor_encoder,
        motor_inverter
    );
    HAL_Delay(1);
    motor->init();

    if (app_config.is_app_running()) {
        motor->start();

        HAL_IMPORTANT(eeprom.read<CalibrationData>(&calibration_data, 0))
        if (calibration_data.type_id != CalibrationData::TYPE_ID || !calibration_data.was_calibrated) {
            calibration_data.reset();
            motor->calibrate(calibration_data);
            calibration_data.was_calibrated = true;
            HAL_IMPORTANT(eeprom.write<CalibrationData>(&calibration_data, 0))
        }
        motor->apply_calibration(calibration_data);
        motor->set_voltage_point(0.0f);

        set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);
    }

    HAL_TIM_Base_Start_IT(&htim1);
    while(true) {}
}

#define NO_CYPHAL
#ifndef NO_CYPHAL
#pragma region Cyphal
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(AngularVelocity, uavcan_si_unit_angular_velocity_Scalar_1_0)
TYPE_ALIAS(Angle, uavcan_si_unit_angle_Scalar_1_0)
TYPE_ALIAS(Torque, uavcan_si_unit_torque_Scalar_1_0)
TYPE_ALIAS(Voltage, uavcan_si_unit_voltage_Scalar_1_0)
static constexpr CanardPortID ANGLE_PORT = 6998;
static constexpr CanardPortID ANGULAR_VELOCITY_PORT = 7006;
static constexpr CanardPortID TORQUE_PORT = 1423;
static constexpr CanardPortID VOLTAGE_PORT = 3423;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 2, {
        /*  // TODO
        Angle::Type angle_msg = {};
        angle_msg.radian = motor->get_angle();
        static CanardTransferID angle_transfer_id = 0;
        get_interface()->send_msg<Angle>(&angle_msg, ANGLE_PORT, &angle_transfer_id);

        AngularVelocity::Type angle_velocity_msg = {};
        angle_velocity_msg.radian_per_second = motor->get_velocity();
        static CanardTransferID angle_velocity_transfer_id = 0;
        get_interface()->send_msg<AngularVelocity>(&angle_velocity_msg, ANGULAR_VELOCITY_PORT, &angle_velocity_transfer_id);
        */
    })
}

class VoltageSub: public AbstractSubscription<Voltage> {
public:
VoltageSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Voltage>(interface, port_id) {};
    void handler(const Voltage::Type& msg, CanardRxTransfer* _) override {
        //motor->set_voltage_point(msg.volt);  // TODO
    }
};

class TorqueSub: public AbstractSubscription<Torque> {
public:
TorqueSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Torque>(interface, port_id) {};
    void handler(const Torque::Type& msg, CanardRxTransfer* _) override {
        //motor->set_torque_point(msg.newton_meter);  // TODO
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
ReservedObject<VoltageSub> voltage_sub;
ReservedObject<TorqueSub> torque_sub;

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

                    //motor->set_state(value);  // TODO

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    //v_out.bit.value.bitpacked[0] = motor->is_on();  // TODO
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    voltage_sub.create(cyphal_interface, VOLTAGE_PORT + node_id);
    torque_sub.create(cyphal_interface, TORQUE_PORT + node_id);
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.vbdrive",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        voltage_sub->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        torque_sub->make_filter(node_id)
    ))
}
#pragma endregion
#endif
