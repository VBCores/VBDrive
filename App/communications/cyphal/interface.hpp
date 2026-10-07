#pragma once
#include "app.h"
#include "config/config.hpp"
#include "state_manager/state_manager.h"
#include "profiling.hpp"
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
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
#include <voltbro/foc/MIT_1_0.hpp>
#include <voltbro/foc/Servo_1_0.hpp>
#include <voltbro/foc/State_1_0.hpp>


extern FDCAN_HandleTypeDef hfdcan1;
extern VBInverter motor_inverter;
std::shared_ptr<CyphalInterface> get_interface();

static constexpr CanardPortID FOC_COMMAND_PORT = 2107;
static constexpr CanardPortID FOC_STATE_PORT = 3811;
static constexpr CanardPortID SERVO_PORT = 3407;

void in_loop_reporting(millis current_t) {
    auto motor = get_motor();
    if (motor == nullptr) {
        return;
    }

    static millis report_time = 0;
    if (current_t != report_time) {
        report_time = current_t; // One fresh sample; never replay overdue slots.
        voltbro_foc_State_1_0 state_msg = {};

        state_msg.timestamp.microsecond = system_time();

        state_msg.pos.radian = motor->get_angle();
        state_msg.vel.radian_per_second = motor->get_velocity();
        state_msg._torq.newton_meter = motor->get_torque();

        // Keep temperature measurements fresh for Serial/Cyphal registers.
        motor_inverter.update_temperature();

        static CanardTransferID state_transfer_id = 0;
        get_interface()->send_msg(&state_msg, FOC_STATE_PORT, &state_transfer_id, 1000);
        VBDRIVE_PROFILE_COUNT(state_messages_queued)
    }
}

class FOCCommandSub: public AbstractSubscription<voltbro_foc_MIT_1_0> {
public:
    FOCCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<voltbro_foc_MIT_1_0>(interface, port_id) {};
    void handler(const voltbro_foc_MIT_1_0& msg, CanardRxTransfer*) override {
        VBDRIVE_PROFILE_HANDLER(mit_handler_timing);
        bool is_valid = apply_mit_command(FOCTarget {
            .torque = msg._torq.newton_meter,
            .angle = msg.pos.radian,
            .velocity = msg.vel.radian_per_second,
            .angle_kp = msg.pos_gain.value,
            .velocity_kp = msg.vel_gain.value
        });
        VBDRIVE_PROFILE_RESULT(mit, is_valid)
        if (!is_valid) {
            record_invalid_command();
        }
    }
};

class ServoSub: public AbstractSubscription<voltbro_foc_Servo_1_0> {
public:
    ServoSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<voltbro_foc_Servo_1_0>(interface, port_id) {};
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    // NOTE: transfer parameter required by the interface, but not used in this implementation
    void handler(const voltbro_foc_Servo_1_0& msg, CanardRxTransfer* _) override {
    #pragma GCC diagnostic pop
        VBDRIVE_PROFILE_HANDLER(servo_handler_timing);
        const bool is_valid = apply_servo_command(msg.control_type, msg.set_point_value,
                                                  msg.command_idx.count != 0,
                                                  msg.command_idx.count ? msg.command_idx.elements[0] : 0);
        if (!is_valid) {
            record_invalid_command();
        }
    }
};

// NOTE: underlying CanardRxSubscriptions are HUGE - 552 bytes each. C++ wrapper size is negligible in comparison
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<PARAMETER_CATALOG.size(), StaticRegisters<PARAMETER_CATALOG.size()>>> registers_handler;
ReservedObject<FOCCommandSub> foc_command_sub;
ReservedObject<ServoSub> servo_sub;

static void handle_parameter_register(
    size_t index,
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse& response
) {
    const auto& definition = PARAMETER_CATALOG[index];
    response.persistent = definition.is_persistent;
    response._mutable = definition.is_mutable;

    if (definition.is_mutable && v_in._tag_ != REGISTER_EMPTY_TAG) {
        ParameterValue requested{};
        bool parsed = false;
        switch (definition.type) {
            case ParameterType::REAL32:
                parsed = parse_register_real32(v_in, requested.emplace<float>());
                break;
            case ParameterType::NATURAL32:
                parsed = parse_register_natural32(v_in, requested.emplace<uint32_t>());
                if (!parsed) {
                    int32_t signed_value = 0;
                    parsed = parse_register_integer32(v_in, signed_value) && signed_value >= 0;
                    if (parsed) requested = static_cast<uint32_t>(signed_value);
                }
                break;
            case ParameterType::INTEGER32:
                parsed = parse_register_integer32(v_in, requested.emplace<int32_t>());
                break;
            case ParameterType::BIT:
                parsed = parse_register_bit(v_in, requested.emplace<bool>());
                if (!parsed) {
                    int32_t signed_value = 0;
                    uint32_t unsigned_value = 0;
                    float real_value = 0;
                    if (parse_register_integer32(v_in, signed_value)) {
                        requested = signed_value == 1;
                        parsed = signed_value == 0 || signed_value == 1;
                    } else if (parse_register_natural32(v_in, unsigned_value)) {
                        requested = unsigned_value == 1;
                        parsed = unsigned_value == 0 || unsigned_value == 1;
                    } else if (parse_register_real32(v_in, real_value)) {
                        requested = real_value == 1;
                        parsed = real_value == 0 || real_value == 1;
                    }
                }
                break;
            case ParameterType::STRING:
                if (v_in._tag_ == REGISTER_STRING_TAG) {
                    requested = std::string_view(
                        reinterpret_cast<const char*>(v_in._string.value.elements),
                        v_in._string.value.count);
                    parsed = true;
                }
                break;
        }
        if (parsed) {
            auto& config = get_app_manager().get_committed_config();
            const auto write_result = definition.is_persistent
                ? write_persistent_parameter(config, definition.id, requested, get_motor() != nullptr)
                : write_runtime_parameter(definition.id, requested);
            if (write_result == ParameterWriteResult::OK && definition.is_persistent) {
                get_app_manager().mark_config_changed();
            }
        }
    }

    ParameterValue current{};
    if (!read_parameter(get_app_manager().get_committed_config(), definition.id, current)) {
        v_out._tag_ = REGISTER_EMPTY_TAG;
        v_out.empty = {};
        return;
    }
    switch (definition.type) {
        case ParameterType::REAL32:
            fill_register_real32(v_out, std::get<float>(current));
            break;
        case ParameterType::NATURAL32:
            fill_register_natural32(v_out, std::get<uint32_t>(current));
            break;
        case ParameterType::INTEGER32:
            fill_register_integer32(v_out, std::get<int32_t>(current));
            break;
        case ParameterType::BIT:
            fill_register_bit(v_out, std::get<bool>(current));
            break;
        case ParameterType::STRING:
            fill_register_string(v_out, std::get<std::string_view>(current));
            break;
    }
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
    registers_handler.create(
        StaticRegisters<PARAMETER_CATALOG.size()>{PARAMETER_NAMES, handle_parameter_register},
        cyphal_interface
    );

    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.vbdrive",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{VBDRIVE_VERSION_MAJOR, VBDRIVE_VERSION_MINOR},
        VBDRIVE_VCS_REVISION_ID
    );

    servo_sub.create(cyphal_interface, SERVO_PORT + node_id);
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
        servo_sub->make_filter(node_id)
    ))
}
