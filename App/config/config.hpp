#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <variant>
#include <optional>
#include <cmath>
#include <voltbro/config/config_manager.hpp>

struct ServoInputConfig;
class EEPROM;
enum class AngleEncoderType : uint8_t;

enum class ParameterType : uint8_t {
    REAL32,
    NATURAL32,
    INTEGER32,
    BIT,
    STRING
};

enum class ParameterId : uint8_t {
    GEAR,
    MAX_I,
    MAX_SPD,
    MAX_TQ,
    ANG_OFF,
    ANG_DIR,
    MIN_ANG,
    MAX_ANG,
    KT,
    KP,
    KI,
    KD,
    FLT_A,
    FLT_G1,
    FLT_G2,
    FLT_G3,
    I_LPF,
    ANG_ENC,
    NODE_ID,
    DATA_BAUD,
    NOMINAL_BAUD,
    IS_ON,
    BOOTLOADER,
    CMD_ERRORS,
    NAME,
    REVISION,
    BUS_VOLTAGE,
    BUS_CURRENT,
    TEMP_MCU,
    TEMP_STATOR,
    IS_FAULT,
    ENCODER_SHAFT,
    ENCODER_ROTOR,
    SERVO_POS_P_GAIN,
    SERVO_POS_I_GAIN,
    SERVO_POS_D_GAIN,
    SERVO_VEL_P_GAIN,
    SERVO_VEL_I_GAIN,
    SERVO_CONTROL_INPUT_BANDWITH,
    SERVO_CONTROL_VEL_LIMIT,
    SERVO_CONTROL_ACCEL_LIMIT,
    SERVO_CONTROL_DECEL_LIMIT,
    SERVO_CONTROL_VEL_RAMP_RATE,
    SERIAL_BAUD,
    DEVICE,
    RATED_MAX_TORQUE,
    RATED_MAX_CURRENT
};

using ParameterValue = std::variant<uint32_t, int32_t, float, bool, std::string_view>;

struct ParameterDefinition {
    ParameterId id;
    std::string_view name;
    ParameterType type;
    bool is_mutable;
    bool is_persistent;
    std::optional<ParameterValue> default_value;

};


enum class ParameterWriteResult : uint8_t {
    OK,
    READ_ONLY,
    INVALID,
    UNAVAILABLE
};

inline constexpr std::array<ParameterDefinition, 47> PARAMETER_CATALOG{{
    {ParameterId::GEAR,          "gear",          ParameterType::NATURAL32, true,  true, ParameterValue{uint32_t{36}}},
    {ParameterId::MAX_I,         "max_i",         ParameterType::REAL32,    true,  true, ParameterValue{float{NAN}}},
    {ParameterId::MAX_SPD,       "max_spd",       ParameterType::REAL32,    true,  true, ParameterValue{float{NAN}}},
    {ParameterId::MAX_TQ,        "max_tq",        ParameterType::REAL32,    true,  true, ParameterValue{float{NAN}}},
    {ParameterId::ANG_OFF,       "ang_off",       ParameterType::REAL32,    true,  true, ParameterValue{float{0.0f}}},
    {ParameterId::ANG_DIR,       "ang_dir",       ParameterType::INTEGER32, true,  true, ParameterValue{int32_t{1}}},
    {ParameterId::MIN_ANG,       "min_ang",       ParameterType::REAL32,    true,  true, ParameterValue{float{NAN}}},
    {ParameterId::MAX_ANG,       "max_ang",       ParameterType::REAL32,    true,  true, ParameterValue{float{NAN}}},
    {ParameterId::KT,            "kt",            ParameterType::REAL32,    true,  true, ParameterValue{float{1.0f}}},
    {ParameterId::KP,            "kp",            ParameterType::REAL32,    true,  true, ParameterValue{float{4.0f}}},
    {ParameterId::KI,            "ki",            ParameterType::REAL32,    true,  true, ParameterValue{float{1600.0f}}},
    {ParameterId::KD,            "kd",            ParameterType::REAL32,    true,  true, ParameterValue{float{0.0f}}},
    {ParameterId::FLT_A,         "flt_a",         ParameterType::REAL32,    true,  true, ParameterValue{float{0.0f}}},
    {ParameterId::FLT_G1,        "flt_g1",        ParameterType::REAL32,    true,  true, ParameterValue{float{0.015700989410003974f}}},
    {ParameterId::FLT_G2,        "flt_g2",        ParameterType::REAL32,    true,  true, ParameterValue{float{3.925227776360174f}}},
    {ParameterId::FLT_G3,        "flt_g3",        ParameterType::REAL32,    true,  true, ParameterValue{float{387.54711795263574f}}},
    {ParameterId::I_LPF,         "i_lpf",         ParameterType::REAL32,    true,  true, ParameterValue{float{0.0925f}}},
    {ParameterId::ANG_ENC,       "ang_enc",       ParameterType::NATURAL32, true,  true, ParameterValue{uint32_t{0}}},
    {ParameterId::NODE_ID,       "node_id",       ParameterType::NATURAL32, true,  true, ParameterValue{uint32_t{0}}},
    {ParameterId::DATA_BAUD,     "data_baud",     ParameterType::NATURAL32, true,  true, ParameterValue{uint32_t{VOLTBRO_DEFAULT_CAN_DATA_BAUD}}},
    {ParameterId::NOMINAL_BAUD,  "nominal_baud",  ParameterType::NATURAL32, true,  true, ParameterValue{uint32_t{VOLTBRO_DEFAULT_CAN_NOMINAL_BAUD}}},
    {ParameterId::IS_ON,         "is_on",         ParameterType::BIT,       true,  false, std::nullopt},
    {ParameterId::BOOTLOADER,    "bootloader",    ParameterType::BIT,       true,  false, std::nullopt},
    {ParameterId::CMD_ERRORS,    "cmd_errors",    ParameterType::NATURAL32, false, false, std::nullopt},
    {ParameterId::NAME,          "name",          ParameterType::STRING,    true,  true, ParameterValue{std::string_view{"vbdrive"}}},
    {ParameterId::REVISION,      "firmware_rev",  ParameterType::STRING,    false, false, std::nullopt},
    {ParameterId::BUS_VOLTAGE,   "bus_voltage",   ParameterType::REAL32,    false, false, std::nullopt},
    {ParameterId::BUS_CURRENT,   "bus_current",   ParameterType::REAL32,    false, false, std::nullopt},
    {ParameterId::TEMP_MCU,      "temp_mcu",      ParameterType::REAL32,    false, false, std::nullopt},
    {ParameterId::TEMP_STATOR,   "temp_stator",   ParameterType::REAL32,    false, false, std::nullopt},
    {ParameterId::IS_FAULT,      "is_fault",      ParameterType::BIT,       false, false, std::nullopt},
    {ParameterId::ENCODER_SHAFT, "encoder_shaft", ParameterType::NATURAL32, false, false, std::nullopt},
    {ParameterId::ENCODER_ROTOR, "encoder_rotor", ParameterType::NATURAL32, false, false, std::nullopt},
    {ParameterId::SERVO_POS_P_GAIN, "servo_pos_p_gain", ParameterType::REAL32, true, true, ParameterValue{float{150.0f}}},
    {ParameterId::SERVO_POS_I_GAIN, "servo_pos_i_gain", ParameterType::REAL32, true, true, ParameterValue{float{200.0f}}},
    {ParameterId::SERVO_POS_D_GAIN, "servo_pos_d_gain", ParameterType::REAL32, true, true, ParameterValue{float{10.0f}}},
    {ParameterId::SERVO_VEL_P_GAIN, "servo_vel_p_gain", ParameterType::REAL32, true, true, ParameterValue{float{30.0f}}},
    {ParameterId::SERVO_VEL_I_GAIN, "servo_vel_i_gain", ParameterType::REAL32, true, true, ParameterValue{float{60.0f}}},
    {ParameterId::SERVO_CONTROL_INPUT_BANDWITH, "servo_control_input_bandwith", ParameterType::REAL32, true, true, ParameterValue{float{0.0f}}},
    {ParameterId::SERVO_CONTROL_VEL_LIMIT, "servo_control_vel_limit", ParameterType::REAL32, true, true, ParameterValue{float{0.0f}}},
    {ParameterId::SERVO_CONTROL_ACCEL_LIMIT, "servo_control_accel_limit", ParameterType::REAL32, true, true, ParameterValue{float{0.0f}}},
    {ParameterId::SERVO_CONTROL_DECEL_LIMIT, "servo_control_decel_limit", ParameterType::REAL32, true, true, ParameterValue{float{0.0f}}},
    {ParameterId::SERVO_CONTROL_VEL_RAMP_RATE, "servo_control_vel_ramp_rate", ParameterType::REAL32, true, true, ParameterValue{float{0.0f}}},
    {ParameterId::SERIAL_BAUD, "serial_baud", ParameterType::NATURAL32, true, true, ParameterValue{uint32_t{VOLTBRO_DEFAULT_SERIAL_BAUD}}},
    {ParameterId::DEVICE, "device", ParameterType::STRING, false, false, std::nullopt},
    {ParameterId::RATED_MAX_TORQUE, "rated_max_torque", ParameterType::REAL32, true, true, ParameterValue{float{30.0f}}},
    {ParameterId::RATED_MAX_CURRENT, "rated_max_current", ParameterType::REAL32, true, true, ParameterValue{float{30.0f}}},
}};

consteval bool parameter_catalog_is_valid() {
    for (size_t i = 0; i < PARAMETER_CATALOG.size(); ++i) {
        const auto& parameter = PARAMETER_CATALOG[i];
        if (parameter.default_value.has_value() != parameter.is_persistent) return false;
        if (parameter.default_value) {
            const size_t expected[] = {2, 0, 1, 3, 4};
            if (parameter.default_value->index() != expected[static_cast<size_t>(parameter.type)]) return false;
        }
        if (static_cast<size_t>(parameter.id) != i) return false;
        if (PARAMETER_CATALOG[i].name.empty()) {
            return false;
        }
        for (size_t j = i + 1; j < PARAMETER_CATALOG.size(); ++j) {
            if (PARAMETER_CATALOG[i].name == PARAMETER_CATALOG[j].name) {
                return false;
            }
        }
    }
    return true;
}

static_assert(parameter_catalog_is_valid());

inline constexpr std::array<std::string_view, PARAMETER_CATALOG.size()> PARAMETER_NAMES = [] {
    std::array<std::string_view, PARAMETER_CATALOG.size()> names{};
    for (size_t i = 0; i < names.size(); ++i) {
        names[i] = PARAMETER_CATALOG[i].name;
    }
    return names;
}();

template<class T>
consteval T parameter_default(ParameterId id) {
    return std::get<T>(*PARAMETER_CATALOG[static_cast<size_t>(id)].default_value);
}

template<class T>
inline T value_or_default(T value, T fallback) {
    return std::isnan(value) ? fallback : value;
}

template<class T>
inline T value_or_default(T value, T fallback, T unset) {
    return value == unset ? fallback : value;
}

inline constexpr BaseConfigData BASE_CONFIG_DEFAULTS = [] {
    BaseConfigData config{};
    config.type_id = VOLTBRO_BASE_CONFIG_TYPE_ID;
    config.serial_baud = parameter_default<uint32_t>(ParameterId::SERIAL_BAUD);
    config.node_id = parameter_default<uint32_t>(ParameterId::NODE_ID);
    config.fdcan_nominal_baud = parameter_default<uint32_t>(ParameterId::NOMINAL_BAUD);
    config.fdcan_data_baud = parameter_default<uint32_t>(ParameterId::DATA_BAUD);
    const auto name = parameter_default<std::string_view>(ParameterId::NAME);
    for (size_t i = 0; i < name.size(); ++i) config.name[i] = name[i];
    return config;
}();

inline constexpr uint32_t VBDRIVE_CONFIG_TYPE_ID = 0x44AAAC03;

struct __attribute__((packed)) VBDriveConfig {
    static constexpr uint32_t TYPE_ID = VBDRIVE_CONFIG_TYPE_ID;
    uint32_t type_id = TYPE_ID;
    uint8_t gear_ratio = 0;
    int32_t angle_direction = parameter_default<int32_t>(ParameterId::ANG_DIR);
    float max_current = NAN;
    float max_torque = NAN;
    float max_speed = NAN;
    float angle_offset = NAN;
    float min_angle = NAN;
    float max_angle = NAN;
    float torque_const = NAN;
    float kp = NAN;
    float ki = NAN;
    float kd = NAN;
    float filter_a = NAN;
    float filter_g1 = NAN;
    float filter_g2 = NAN;
    float filter_g3 = NAN;
    float I_lpf_coefficient = NAN;
    AngleEncoderType angle_encoder = static_cast<AngleEncoderType>(parameter_default<uint32_t>(ParameterId::ANG_ENC));
    float servo_pos_p_gain = NAN;
    float servo_pos_i_gain = NAN;
    float servo_pos_d_gain = NAN;
    float servo_vel_p_gain = NAN;
    float servo_vel_i_gain = NAN;
    float servo_control_input_bandwith = NAN;
    float servo_control_vel_limit = NAN;
    float servo_control_accel_limit = NAN;
    float servo_control_decel_limit = NAN;
    float servo_control_vel_ramp_rate = NAN;
    float rated_max_torque = NAN;
    float rated_max_current = NAN;

    bool are_required_params_set(const BaseConfigData& base) const;
    void apply_servo_config() const;
    ServoInputConfig servo_input_config() const;
};

using DriveConfigManager = ConfigManager<VBDriveConfig, EEPROM>;
using DriveConfig = DriveConfigManager::Data;
#ifndef VB_CONFIG_ADDRESS
#define VB_CONFIG_ADDRESS 0
#endif
inline constexpr uint32_t CONFIG_PLACEMENT = VB_CONFIG_ADDRESS;
inline constexpr size_t CALIBRATION_PLACEMENT = 0x0100;
inline constexpr size_t IND_SENSOR_STATE_PLACEMENT = 0x2200;
static_assert(sizeof(BaseConfigData) + sizeof(VBDriveConfig) == sizeof(DriveConfig));
static_assert(VB_CONFIG_ADDRESS >= 0 && VB_CONFIG_ADDRESS <= 32768U - sizeof(DriveConfig), "Configuration exceeds EEPROM capacity");

const ParameterDefinition* find_parameter(std::string_view name);
bool read_parameter(const DriveConfig& config, ParameterId id, ParameterValue& value);
ParameterWriteResult write_persistent_parameter(DriveConfig& config, ParameterId id,
                                               const ParameterValue& value, bool apply_runtime);
ParameterWriteResult write_runtime_parameter(ParameterId id, const ParameterValue& value);
void record_invalid_command();
inline bool bootloader_reboot_pending = false;
