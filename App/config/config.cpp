#include "main.h"
#include "app.h"
#include "config.hpp"
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <cmath>

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

bool read_parameter(const DriveConfig& data, ParameterId id, ParameterValue& value) {
    const auto& config = data.app;
    const auto& base = data.base;
    value = {};
    switch (id) {
        case ParameterId::SERVO_POS_P_GAIN: value = value_or_default(config.servo_pos_p_gain, parameter_default<float>(ParameterId::SERVO_POS_P_GAIN)); return true;
        case ParameterId::SERVO_POS_I_GAIN: value = value_or_default(config.servo_pos_i_gain, parameter_default<float>(ParameterId::SERVO_POS_I_GAIN)); return true;
        case ParameterId::SERVO_POS_D_GAIN: value = value_or_default(config.servo_pos_d_gain, parameter_default<float>(ParameterId::SERVO_POS_D_GAIN)); return true;
        case ParameterId::SERVO_VEL_P_GAIN: value = value_or_default(config.servo_vel_p_gain, parameter_default<float>(ParameterId::SERVO_VEL_P_GAIN)); return true;
        case ParameterId::SERVO_VEL_I_GAIN: value = value_or_default(config.servo_vel_i_gain, parameter_default<float>(ParameterId::SERVO_VEL_I_GAIN)); return true;
        case ParameterId::SERVO_CONTROL_INPUT_BANDWITH: value = value_or_default(config.servo_control_input_bandwith, parameter_default<float>(ParameterId::SERVO_CONTROL_INPUT_BANDWITH)); return true;
        case ParameterId::SERVO_CONTROL_VEL_LIMIT: value = value_or_default(config.servo_control_vel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_VEL_LIMIT)); return true;
        case ParameterId::SERVO_CONTROL_ACCEL_LIMIT: value = value_or_default(config.servo_control_accel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_ACCEL_LIMIT)); return true;
        case ParameterId::SERVO_CONTROL_DECEL_LIMIT: value = value_or_default(config.servo_control_decel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_DECEL_LIMIT)); return true;
        case ParameterId::SERVO_CONTROL_VEL_RAMP_RATE: value = value_or_default(config.servo_control_vel_ramp_rate, parameter_default<float>(ParameterId::SERVO_CONTROL_VEL_RAMP_RATE)); return true;
        case ParameterId::ANG_DIR:
            value.emplace<int32_t>(config.angle_direction == -1 ? -1 : 1);
            return true;
        case ParameterId::BOOTLOADER:
            value.emplace<bool>(bootloader_reboot_pending);
            return true;
        case ParameterId::GEAR:
            value.emplace<uint32_t>(value_or_default(config.gear_ratio, static_cast<uint8_t>(parameter_default<uint32_t>(ParameterId::GEAR)), static_cast<uint8_t>(0)));
            return true;
        case ParameterId::MAX_I:
            value.emplace<float>(value_or_default(config.max_current, parameter_default<float>(ParameterId::MAX_I)));
            return true;
        case ParameterId::MAX_SPD:
            value.emplace<float>(value_or_default(config.max_speed, parameter_default<float>(ParameterId::MAX_SPD)));
            return true;
        case ParameterId::MAX_TQ:
            value.emplace<float>(value_or_default(config.max_torque, parameter_default<float>(ParameterId::MAX_TQ)));
            return true;
        case ParameterId::RATED_MAX_TORQUE:
            value.emplace<float>(value_or_default(config.rated_max_torque, parameter_default<float>(ParameterId::RATED_MAX_TORQUE)));
            return true;
        case ParameterId::RATED_MAX_CURRENT:
            value.emplace<float>(value_or_default(config.rated_max_current, parameter_default<float>(ParameterId::RATED_MAX_CURRENT)));
            return true;
        case ParameterId::ANG_OFF:
            value.emplace<float>(value_or_default(config.angle_offset, parameter_default<float>(ParameterId::ANG_OFF)));
            return true;
        case ParameterId::MIN_ANG:
            value.emplace<float>(value_or_default(config.min_angle, parameter_default<float>(ParameterId::MIN_ANG)));
            return true;
        case ParameterId::MAX_ANG:
            value.emplace<float>(value_or_default(config.max_angle, parameter_default<float>(ParameterId::MAX_ANG)));
            return true;
        case ParameterId::KT:
            value.emplace<float>(value_or_default(config.torque_const, parameter_default<float>(ParameterId::KT)));
            return true;
        case ParameterId::KP:
            value.emplace<float>(value_or_default(config.kp, parameter_default<float>(ParameterId::KP)));
            return true;
        case ParameterId::KI:
            value.emplace<float>(value_or_default(config.ki, parameter_default<float>(ParameterId::KI)));
            return true;
        case ParameterId::KD:
            value.emplace<float>(value_or_default(config.kd, parameter_default<float>(ParameterId::KD)));
            return true;
        case ParameterId::FLT_A:
            value.emplace<float>(value_or_default(config.filter_a, parameter_default<float>(ParameterId::FLT_A)));
            return true;
        case ParameterId::FLT_G1:
            value.emplace<float>(value_or_default(config.filter_g1, parameter_default<float>(ParameterId::FLT_G1)));
            return true;
        case ParameterId::FLT_G2:
            value.emplace<float>(value_or_default(config.filter_g2, parameter_default<float>(ParameterId::FLT_G2)));
            return true;
        case ParameterId::FLT_G3:
            value.emplace<float>(value_or_default(config.filter_g3, parameter_default<float>(ParameterId::FLT_G3)));
            return true;
        case ParameterId::I_LPF:
            value.emplace<float>(value_or_default(config.I_lpf_coefficient, parameter_default<float>(ParameterId::I_LPF)));
            return true;
        case ParameterId::ANG_ENC:
            value.emplace<uint32_t>(to_underlying(config.angle_encoder));
            return true;
        case ParameterId::NODE_ID:
            value.emplace<uint32_t>(base.node_id);
            return true;
        case ParameterId::DATA_BAUD:
            value.emplace<uint32_t>(base.fdcan_data_baud);
            return true;
        case ParameterId::NOMINAL_BAUD:
            value.emplace<uint32_t>(base.fdcan_nominal_baud);
            return true;
        case ParameterId::SERIAL_BAUD:
            value.emplace<uint32_t>(base.serial_baud);
            return true;
        case ParameterId::CMD_ERRORS:
            value.emplace<uint32_t>(invalid_commands_counter);
            return true;
        case ParameterId::NAME:
            value.emplace<std::string_view>(base.name, strnlen(base.name, sizeof(base.name)));
            return true;
        case ParameterId::REVISION:
            value.emplace<std::string_view>(VBDRIVE_FIRMWARE_REV);
            return true;
        case ParameterId::DEVICE:
            value.emplace<std::string_view>("vbdrive");
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

[[gnu::optimize("Os")]] ParameterWriteResult write_persistent_parameter(
    DriveConfig& data,
    ParameterId id,
    const ParameterValue& value,
    bool apply_runtime
) {
    auto& config = data.app;
    auto& base = data.base;
    if (id == ParameterId::RATED_MAX_TORQUE || id == ParameterId::RATED_MAX_CURRENT) {
        const float rating = value_or_default(std::get<float>(value), id == ParameterId::RATED_MAX_CURRENT
            ? parameter_default<float>(ParameterId::RATED_MAX_CURRENT)
            : parameter_default<float>(ParameterId::RATED_MAX_TORQUE));
        if (!std::isfinite(rating) || rating <= 0) return ParameterWriteResult::INVALID;
        if (id == ParameterId::RATED_MAX_CURRENT) config.rated_max_current = std::get<float>(value);
        else config.rated_max_torque = std::get<float>(value);
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::SERIAL_BAUD) {
        const auto baud = std::get<uint32_t>(value);
        if (!voltbro_serial_baud_valid(baud)) return ParameterWriteResult::INVALID;
        base.serial_baud = baud;
        return ParameterWriteResult::OK;
    }
    if (id == ParameterId::NAME) {
        const auto name = std::get<std::string_view>(value);
        if (name.empty() || name.size() >= sizeof(base.name)) return ParameterWriteResult::INVALID;
        for (unsigned char byte : name) {
            if (byte < 0x20 || byte == 0x7F) return ParameterWriteResult::INVALID;
        }
        memset(base.name, 0, sizeof(base.name));
        memcpy(base.name, name.data(), name.size());
        return ParameterWriteResult::OK;
    }
    if (id >= ParameterId::SERVO_POS_P_GAIN && id <= ParameterId::SERVO_CONTROL_VEL_RAMP_RATE) {
        const float gain = std::get<float>(value);
        const bool generator_parameter = id >= ParameterId::SERVO_CONTROL_INPUT_BANDWITH;
        if ((!std::isfinite(gain) && !(generator_parameter && std::isnan(gain))) || gain < 0) {
            return ParameterWriteResult::INVALID;
        }
        if (generator_parameter) {
            VBDriveConfig candidate = config;
            switch (id) {
                case ParameterId::SERVO_CONTROL_INPUT_BANDWITH: candidate.servo_control_input_bandwith = gain; break;
                case ParameterId::SERVO_CONTROL_VEL_LIMIT: candidate.servo_control_vel_limit = gain; break;
                case ParameterId::SERVO_CONTROL_ACCEL_LIMIT: candidate.servo_control_accel_limit = gain; break;
                case ParameterId::SERVO_CONTROL_DECEL_LIMIT: candidate.servo_control_decel_limit = gain; break;
                case ParameterId::SERVO_CONTROL_VEL_RAMP_RATE: candidate.servo_control_vel_ramp_rate = gain; break;
                default: break;
            }
            if (apply_runtime) {
                auto motor = get_motor();
                if (!motor) return ParameterWriteResult::UNAVAILABLE;
                if (!motor->set_servo_input_config(candidate.servo_input_config())) return ParameterWriteResult::INVALID;
            }
            config = candidate;
        } else {
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
            if (std::get<uint32_t>(value) == 0 || std::get<uint32_t>(value) > 127U) return ParameterWriteResult::INVALID;
            base.node_id = static_cast<uint8_t>(std::get<uint32_t>(value));
            break;
        case ParameterId::DATA_BAUD:
            if (std::get<uint32_t>(value) > 3U) return ParameterWriteResult::INVALID;
            base.fdcan_data_baud = static_cast<uint8_t>(std::get<uint32_t>(value));
            break;
        case ParameterId::NOMINAL_BAUD:
            if (std::get<uint32_t>(value) > 4U) return ParameterWriteResult::INVALID;
            base.fdcan_nominal_baud = static_cast<uint8_t>(std::get<uint32_t>(value));
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
    const bool enabled = std::get<bool>(value);
    if (!enabled) motor->reset_servo_input();
    return motor->set_state(enabled) == HAL_OK ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
}

bool VBDriveConfig::are_required_params_set(const BaseConfigData& base) const {
    return base.node_id != 0 && gear_ratio != 0;
}

ServoInputConfig VBDriveConfig::servo_input_config() const {
    return {
        .input_bandwidth = value_or_default(servo_control_input_bandwith, parameter_default<float>(ParameterId::SERVO_CONTROL_INPUT_BANDWITH)),
        .velocity_limit = value_or_default(servo_control_vel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_VEL_LIMIT)),
        .acceleration_limit = value_or_default(servo_control_accel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_ACCEL_LIMIT)),
        .deceleration_limit = value_or_default(servo_control_decel_limit, parameter_default<float>(ParameterId::SERVO_CONTROL_DECEL_LIMIT)),
        .velocity_ramp_rate = value_or_default(servo_control_vel_ramp_rate, parameter_default<float>(ParameterId::SERVO_CONTROL_VEL_RAMP_RATE))
    };
}

void VBDriveConfig::apply_servo_config() const {
    auto motor = get_motor();
    if (!motor) return;
    motor->update_servo_config(SetPointType::POSITION,
        PIDConfig{.kp = value_or_default(servo_pos_p_gain, parameter_default<float>(ParameterId::SERVO_POS_P_GAIN)),
                  .ki = value_or_default(servo_pos_i_gain, parameter_default<float>(ParameterId::SERVO_POS_I_GAIN)),
                  .kd = value_or_default(servo_pos_d_gain, parameter_default<float>(ParameterId::SERVO_POS_D_GAIN))});
    motor->update_servo_config(SetPointType::VELOCITY,
        PIDConfig{.kp = value_or_default(servo_vel_p_gain, parameter_default<float>(ParameterId::SERVO_VEL_P_GAIN)),
                  .ki = value_or_default(servo_vel_i_gain, parameter_default<float>(ParameterId::SERVO_VEL_I_GAIN))});
    motor->set_servo_input_config(servo_input_config());
}
