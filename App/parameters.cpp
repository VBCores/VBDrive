#include "app.h"
#include "parameters.hpp"
#include <cmath>

#ifndef VBDRIVE_MODEL
#error "VBDRIVE_MODEL must be defined by CMake"
#endif

#ifndef VBDRIVE_FIRMWARE_REV
#error "VBDRIVE_FIRMWARE_REV must be defined by CMake"
#endif

static uint32_t invalid_commands_counter = 0;

const ParameterDefinition* find_parameter(std::string_view name) {
    for (const auto& parameter : PARAMETER_CATALOG) {
        if (parameter.name == name) {
            return &parameter;
        }
    }
    return nullptr;
}

void record_invalid_command() {
    invalid_commands_counter += 1;
}

bool read_parameter(const VBDriveConfig& config, ParameterId id, ParameterValue& value) {
    value = {};
    switch (id) {
        case ParameterId::SERVO_POS_P_GAIN: value = value_or_default(config.servo_pos_p_gain, VBDriveDefaults::SERVO_POS_P_GAIN); return true;
        case ParameterId::SERVO_POS_I_GAIN: value = value_or_default(config.servo_pos_i_gain, VBDriveDefaults::SERVO_POS_I_GAIN); return true;
        case ParameterId::SERVO_POS_D_GAIN: value = value_or_default(config.servo_pos_d_gain, VBDriveDefaults::SERVO_POS_D_GAIN); return true;
        case ParameterId::SERVO_VEL_P_GAIN: value = value_or_default(config.servo_vel_p_gain, VBDriveDefaults::SERVO_VEL_P_GAIN); return true;
        case ParameterId::SERVO_VEL_I_GAIN: value = value_or_default(config.servo_vel_i_gain, VBDriveDefaults::SERVO_VEL_I_GAIN); return true;
        case ParameterId::SERVO_TR_FORM: value = value_or_default(config.servo_transient_form, VBDriveDefaults::SERVO_TRANSIENT_FORM, uint32_t(0)); return true;
        case ParameterId::SERVO_TR_VEL: value = value_or_default(config.servo_transient_vel, VBDriveDefaults::SERVO_TRANSIENT_VEL); return true;
        case ParameterId::ANG_DIR:
            value.emplace<int32_t>(config.angle_direction == -1 ? -1 : 1);
            return true;
        case ParameterId::BOOTLOADER:
            value.emplace<bool>(bootloader_reboot_pending);
            return true;
        case ParameterId::GEAR:
            value.emplace<uint32_t>(value_or_default(config.gear_ratio, VBDriveDefaults::GEAR_RATIO, static_cast<uint8_t>(0)));
            return true;
        case ParameterId::MAX_I:
            value.emplace<float>(value_or_default(config.max_current, NAN));
            return true;
        case ParameterId::MAX_SPD:
            value.emplace<float>(value_or_default(config.max_speed, NAN));
            return true;
        case ParameterId::MAX_TQ:
            value.emplace<float>(value_or_default(config.max_torque, NAN));
            return true;
        case ParameterId::ANG_OFF:
            value.emplace<float>(value_or_default(config.angle_offset, VBDriveDefaults::ANGLE_OFFSET));
            return true;
        case ParameterId::MIN_ANG:
            value.emplace<float>(value_or_default(config.min_angle, NAN));
            return true;
        case ParameterId::MAX_ANG:
            value.emplace<float>(value_or_default(config.max_angle, NAN));
            return true;
        case ParameterId::KT:
            value.emplace<float>(value_or_default(config.torque_const, VBDriveDefaults::TORQUE_CONST));
            return true;
        case ParameterId::KP:
            value.emplace<float>(value_or_default(config.kp, VBDriveDefaults::PID_KP));
            return true;
        case ParameterId::KI:
            value.emplace<float>(value_or_default(config.ki, VBDriveDefaults::PID_KI));
            return true;
        case ParameterId::KD:
            value.emplace<float>(value_or_default(config.kd, VBDriveDefaults::PID_KD));
            return true;
        case ParameterId::FLT_A:
            value.emplace<float>(value_or_default(config.filter_a, VBDriveDefaults::FILTER_A));
            return true;
        case ParameterId::FLT_G1:
            value.emplace<float>(value_or_default(config.filter_g1, VBDriveDefaults::FILTER_G1));
            return true;
        case ParameterId::FLT_G2:
            value.emplace<float>(value_or_default(config.filter_g2, VBDriveDefaults::FILTER_G2));
            return true;
        case ParameterId::FLT_G3:
            value.emplace<float>(value_or_default(config.filter_g3, VBDriveDefaults::FILTER_G3));
            return true;
        case ParameterId::I_LPF:
            value.emplace<float>(value_or_default(config.I_lpf_coefficient, VBDriveDefaults::I_LPF));
            return true;
        case ParameterId::ANG_ENC:
            value.emplace<uint32_t>(to_underlying(config.angle_encoder));
            return true;
        case ParameterId::NODE_ID:
            value.emplace<uint32_t>(config.node_id);
            return true;
        case ParameterId::DATA_BAUD:
            value.emplace<uint32_t>(to_underlying(config.fdcan_data_baud));
            return true;
        case ParameterId::NOMINAL_BAUD:
            value.emplace<uint32_t>(to_underlying(config.fdcan_nominal_baud));
            return true;
        case ParameterId::CMD_ERRORS:
            value.emplace<uint32_t>(invalid_commands_counter);
            return true;
        case ParameterId::MODEL:
            value.emplace<std::string_view>(VBDRIVE_MODEL);
            return true;
        case ParameterId::REVISION:
            value.emplace<std::string_view>(VBDRIVE_FIRMWARE_REV);
            return true;
        case ParameterId::IS_FAULT:
            // DRV_FAULT integration is deferred, matching main telemetry.
            value.emplace<bool>(false);
            return true;
        default:
            break;
    }

    auto motor = get_motor();
    if (!motor) {
        return false;
    }
    const auto& inverter = static_cast<const VBInverter&>(motor->get_inverter());
    switch (id) {
        case ParameterId::IS_ON:
            value.emplace<bool>(motor->is_on());
            return true;
        case ParameterId::BUS_VOLTAGE:
            value.emplace<float>(motor->get_voltage());
            return true;
        case ParameterId::BUS_CURRENT:
            value.emplace<float>(motor->get_working_current());
            return true;
        case ParameterId::TEMP_MCU:
            value.emplace<float>(inverter.get_mcu_temperature() + 273.15f);
            return true;
        case ParameterId::TEMP_STATOR:
            value.emplace<float>(inverter.get_stator_temperature() + 273.15f);
            return true;
        case ParameterId::ENCODER_SHAFT:
            value.emplace<uint32_t>(motor->get_shaft_encoder_value());
            return true;
        case ParameterId::ENCODER_ROTOR:
            value.emplace<uint32_t>(motor->get_rotor_encoder_value());
            return true;
        default:
            return false;
    }
}

ParameterWriteResult write_persistent_parameter(
    VBDriveConfig& config,
    ParameterId id,
    const ParameterValue& value,
    bool apply_runtime
) {
    if (id >= ParameterId::SERVO_POS_P_GAIN && id <= ParameterId::SERVO_TR_VEL) {
        if (id == ParameterId::SERVO_TR_FORM) {
            const auto form = std::get<uint32_t>(value);
            if (form != 1 && form != 2) return ParameterWriteResult::INVALID;
            config.servo_transient_form = form;
        } else {
            const float gain = std::get<float>(value);
            if (!std::isfinite(gain) || gain < 0) return ParameterWriteResult::INVALID;
            if (apply_runtime && id <= ParameterId::SERVO_VEL_I_GAIN) {
                auto motor = get_motor();
                if (!motor) return ParameterWriteResult::UNAVAILABLE;
                const auto type = id <= ParameterId::SERVO_POS_D_GAIN ? SetPointType::POSITION : SetPointType::VELOCITY;
                auto active = motor->get_servo_config(type);
                switch (id) {
                    case ParameterId::SERVO_POS_P_GAIN:
                    case ParameterId::SERVO_VEL_P_GAIN: active.kp = gain; break;
                    case ParameterId::SERVO_POS_I_GAIN:
                    case ParameterId::SERVO_VEL_I_GAIN: active.ki = gain; break;
                    case ParameterId::SERVO_POS_D_GAIN: active.kd = gain; break;
                    default: break;
                }
                motor->update_servo_config(type, active);
            }
            switch (id) {
                case ParameterId::SERVO_POS_P_GAIN: config.servo_pos_p_gain = gain; break;
                case ParameterId::SERVO_POS_I_GAIN: config.servo_pos_i_gain = gain; break;
                case ParameterId::SERVO_POS_D_GAIN: config.servo_pos_d_gain = gain; break;
                case ParameterId::SERVO_VEL_P_GAIN: config.servo_vel_p_gain = gain; break;
                case ParameterId::SERVO_VEL_I_GAIN: config.servo_vel_i_gain = gain; break;
                case ParameterId::SERVO_TR_VEL: config.servo_transient_vel = gain; break;
                default: break;
            }
        }
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::ANG_DIR && std::get<int32_t>(value) != -1 && std::get<int32_t>(value) != 1) {
        return ParameterWriteResult::INVALID;
    }
    DriveRuntimeConfig limits{};
    bool changes_limits = false;
    if (apply_runtime) {
        auto motor = get_motor();
        if (!motor) {
            return ParameterWriteResult::UNAVAILABLE;
        }
        limits = motor->get_runtime_config();
        switch (id) {
            case ParameterId::ANG_DIR: limits.user_angle_direction = static_cast<int8_t>(std::get<int32_t>(value)); changes_limits = true; break;
            case ParameterId::MAX_I:   limits.user_current_limit = std::get<float>(value); changes_limits = true; break;
            case ParameterId::MAX_SPD: limits.user_speed_limit = std::get<float>(value); changes_limits = true; break;
            case ParameterId::MAX_TQ:  limits.user_torque_limit = std::get<float>(value); changes_limits = true; break;
            case ParameterId::ANG_OFF: limits.user_angle_offset = std::get<float>(value); changes_limits = true; break;
            case ParameterId::MIN_ANG: limits.user_position_lower_limit = std::get<float>(value); changes_limits = true; break;
            case ParameterId::MAX_ANG: limits.user_position_upper_limit = std::get<float>(value); changes_limits = true; break;
            default: break;
        }
        if (changes_limits && !motor->set_runtime_config(limits)) {
            return ParameterWriteResult::INVALID;
        }
    }

    switch (id) {
        case ParameterId::ANG_DIR:
            config.angle_direction = std::get<int32_t>(value);
            break;
        case ParameterId::GEAR:
            if (std::get<uint32_t>(value) == 0 || std::get<uint32_t>(value) > UINT8_MAX) return ParameterWriteResult::INVALID;
            config.gear_ratio = static_cast<uint8_t>(std::get<uint32_t>(value));
            break;
        case ParameterId::MAX_I: config.max_current = std::get<float>(value); break;
        case ParameterId::MAX_SPD: config.max_speed = std::get<float>(value); break;
        case ParameterId::MAX_TQ: config.max_torque = std::get<float>(value); break;
        case ParameterId::ANG_OFF: config.angle_offset = std::get<float>(value); break;
        case ParameterId::MIN_ANG: config.min_angle = std::get<float>(value); break;
        case ParameterId::MAX_ANG: config.max_angle = std::get<float>(value); break;
        case ParameterId::KT: config.torque_const = std::get<float>(value); break;
        case ParameterId::KP: config.kp = std::get<float>(value); break;
        case ParameterId::KI: config.ki = std::get<float>(value); break;
        case ParameterId::KD: config.kd = std::get<float>(value); break;
        case ParameterId::FLT_A: config.filter_a = std::get<float>(value); break;
        case ParameterId::FLT_G1: config.filter_g1 = std::get<float>(value); break;
        case ParameterId::FLT_G2: config.filter_g2 = std::get<float>(value); break;
        case ParameterId::FLT_G3: config.filter_g3 = std::get<float>(value); break;
        case ParameterId::I_LPF: config.I_lpf_coefficient = std::get<float>(value); break;
        case ParameterId::ANG_ENC:
            if (std::get<uint32_t>(value) > to_underlying(AngleEncoderType::SHAFT)) return ParameterWriteResult::INVALID;
            config.angle_encoder = static_cast<AngleEncoderType>(std::get<uint32_t>(value));
            break;
        case ParameterId::NODE_ID:
            if (std::get<uint32_t>(value) == 0 || std::get<uint32_t>(value) > CANARD_NODE_ID_MAX) return ParameterWriteResult::INVALID;
            config.node_id = static_cast<CanardNodeID>(std::get<uint32_t>(value));
            break;
        case ParameterId::DATA_BAUD:
            if (std::get<uint32_t>(value) > to_underlying(FDCANDataBaud::KHz8000)) return ParameterWriteResult::INVALID;
            config.fdcan_data_baud = static_cast<FDCANDataBaud>(std::get<uint32_t>(value));
            break;
        case ParameterId::NOMINAL_BAUD:
            if (std::get<uint32_t>(value) > to_underlying(FDCANNominalBaud::KHz1000)) return ParameterWriteResult::INVALID;
            config.fdcan_nominal_baud = static_cast<FDCANNominalBaud>(std::get<uint32_t>(value));
            break;
        default:
            return ParameterWriteResult::READ_ONLY;
    }
    return ParameterWriteResult::OK;
}

ParameterWriteResult write_runtime_parameter(ParameterId id, const ParameterValue& value) {
    if (id == ParameterId::BOOTLOADER) {
        bootloader_reboot_pending |= std::get<bool>(value);
        return ParameterWriteResult::OK;
    }
    if (id != ParameterId::IS_ON) {
        return ParameterWriteResult::READ_ONLY;
    }
    auto motor = get_motor();
    if (!motor) {
        return ParameterWriteResult::UNAVAILABLE;
    }
    return motor->set_state(std::get<bool>(value)) == HAL_OK
        ? ParameterWriteResult::OK
        : ParameterWriteResult::INVALID;
}
