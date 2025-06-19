#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "fdcan.h"
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
#include <voltbro/encoders/ASxxxx/AS5048A.hpp>
#include <voltbro/utils.hpp>

//EEPROM eeprom(&hi2c4);  // TODO

void setup_cordic() {
    /*  // TODO
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
    */
}

/*  // TODO
AS5048A encoder(
    GpioPin(
        SPI3_NSS_GPIO_Port,
        SPI3_NSS_Pin
    ),
    &hspi3
    // correct is_inverted and elec_offset values will be set by apply_calibration
);
std::unique_ptr<FOC> motor;

std::byte calib_data_raw_memory[sizeof(CalibrationData)];
CalibrationData* calib_data = reinterpret_cast<CalibrationData*>(calib_data_raw_memory);
*/


[[noreturn]] void app() {
    setup_cordic();
    start_timers();
    start_cyphal();

    /*  // TODO
    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    motor = std::make_unique<FOC>(
        0.00005f,
        // Main control regulator (velocity and position)
        PIDConfig {},
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
            .user_current_limit = 10.0f,
        },
        DriveInfo {
            .torque_const = 0.42f,  // Nm / A == V / (rad/s)
            .max_current = 10.0f,
            .max_torque = 10.0f,
            .stall_current = 6.0f,
            .stall_timeout = 3.0f,
            .stall_tolerance = 0.2f,
            .calibration_voltage = 0.1f,
            .l_pins = {
                GpioPin(INLA_GPIO_Port, INLA_Pin),
                GpioPin(INLB_GPIO_Port, INLB_Pin),
                GpioPin(INLC_GPIO_Port, INLC_Pin)
            },
            .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
            .common = {
                .ppairs = 15,
                .gear_ratio = 1
            }
        },
        &htim1,
        &hadc1,
        encoder
    );
    encoder.start_streaming();
    HAL_Delay(1);
    motor->init();
    motor->start();

    HAL_IMPORTANT(eeprom.read<CalibrationData>(calib_data, 0))
    if (calib_data->type_id != CalibrationData::TYPE_ID || !calib_data->was_calibrated) {
        calib_data->type_id = CalibrationData::TYPE_ID;
        calib_data->was_calibrated = false;
        calib_data->ppair_counter = 0;
        calib_data->meas_elec_offset = 0;
        for (size_t i = 0; i < CALIBRATION_BUFF_SIZE; i++) {
            calib_data->calibration_array[i] = 0;
        }
        motor->calibrate(calib_data);

        calib_data->was_calibrated = true;
        HAL_IMPORTANT(eeprom.write<CalibrationData>(calib_data, 0))
    }
    motor->apply_calibration(calib_data);
    motor->set_voltage_point(0.0f);
    */

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    HAL_TIM_Base_Start_IT(&htim1);
    while(true) {
        cyphal_loop();
    }
}

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
    const auto node_id = get_node_id();

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
