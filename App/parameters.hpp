#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <variant>

struct VBDriveConfig;

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
    MODEL,
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
    SERVO_VEL_P_GAIN,
    SERVO_VEL_I_GAIN,
    SERVO_TR_FORM,
    SERVO_TR_VEL
};

struct ParameterDefinition {
    ParameterId id;
    std::string_view name;
    ParameterType type;
    bool is_mutable;
    bool is_persistent;

    constexpr bool available_in_serial() const {
        return id != ParameterId::BOOTLOADER && id != ParameterId::CMD_ERRORS;
    }
};

using ParameterValue = std::variant<uint32_t, int32_t, float, bool, std::string_view>;

enum class ParameterWriteResult : uint8_t {
    OK,
    READ_ONLY,
    INVALID,
    UNAVAILABLE
};

inline constexpr std::array<ParameterDefinition, 39> PARAMETER_CATALOG{{
    {ParameterId::GEAR,          "gear",          ParameterType::NATURAL32, true,  true},
    {ParameterId::MAX_I,         "max_i",         ParameterType::REAL32,    true,  true},
    {ParameterId::MAX_SPD,       "max_spd",       ParameterType::REAL32,    true,  true},
    {ParameterId::MAX_TQ,        "max_tq",        ParameterType::REAL32,    true,  true},
    {ParameterId::ANG_OFF,       "ang_off",       ParameterType::REAL32,    true,  true},
    {ParameterId::ANG_DIR,       "ang_dir",       ParameterType::INTEGER32, true,  true},
    {ParameterId::MIN_ANG,       "min_ang",       ParameterType::REAL32,    true,  true},
    {ParameterId::MAX_ANG,       "max_ang",       ParameterType::REAL32,    true,  true},
    {ParameterId::KT,            "kt",            ParameterType::REAL32,    true,  true},
    {ParameterId::KP,            "kp",            ParameterType::REAL32,    true,  true},
    {ParameterId::KI,            "ki",            ParameterType::REAL32,    true,  true},
    {ParameterId::KD,            "kd",            ParameterType::REAL32,    true,  true},
    {ParameterId::FLT_A,         "flt_a",         ParameterType::REAL32,    true,  true},
    {ParameterId::FLT_G1,        "flt_g1",        ParameterType::REAL32,    true,  true},
    {ParameterId::FLT_G2,        "flt_g2",        ParameterType::REAL32,    true,  true},
    {ParameterId::FLT_G3,        "flt_g3",        ParameterType::REAL32,    true,  true},
    {ParameterId::I_LPF,         "i_lpf",         ParameterType::REAL32,    true,  true},
    {ParameterId::ANG_ENC,       "ang_enc",       ParameterType::NATURAL32, true,  true},
    {ParameterId::NODE_ID,       "node_id",       ParameterType::NATURAL32, true,  true},
    {ParameterId::DATA_BAUD,     "data_baud",     ParameterType::NATURAL32, true,  true},
    {ParameterId::NOMINAL_BAUD,  "nominal_baud",  ParameterType::NATURAL32, true,  true},
    {ParameterId::IS_ON,         "is_on",         ParameterType::BIT,       true,  false},
    {ParameterId::BOOTLOADER,    "bootloader",    ParameterType::BIT,       true,  false},
    {ParameterId::CMD_ERRORS,    "cmd_errors",    ParameterType::NATURAL32, false, false},
    {ParameterId::MODEL,         "vbdrive_model", ParameterType::STRING,    false, false},
    {ParameterId::REVISION,      "firmware_rev",  ParameterType::STRING,    false, false},
    {ParameterId::BUS_VOLTAGE,   "bus_voltage",   ParameterType::REAL32,    false, false},
    {ParameterId::BUS_CURRENT,   "bus_current",   ParameterType::REAL32,    false, false},
    {ParameterId::TEMP_MCU,      "temp_mcu",      ParameterType::REAL32,    false, false},
    {ParameterId::TEMP_STATOR,   "temp_stator",   ParameterType::REAL32,    false, false},
    {ParameterId::IS_FAULT,      "is_fault",      ParameterType::BIT,       false, false},
    {ParameterId::ENCODER_SHAFT, "encoder_shaft", ParameterType::NATURAL32, false, false},
    {ParameterId::ENCODER_ROTOR, "encoder_rotor", ParameterType::NATURAL32, false, false},
    {ParameterId::SERVO_POS_P_GAIN, "servo_pos_p_gain", ParameterType::REAL32, true, true},
    {ParameterId::SERVO_POS_I_GAIN, "servo_pos_i_gain", ParameterType::REAL32, true, true},
    {ParameterId::SERVO_VEL_P_GAIN, "servo_vel_p_gain", ParameterType::REAL32, true, true},
    {ParameterId::SERVO_VEL_I_GAIN, "servo_vel_i_gain", ParameterType::REAL32, true, true},
    {ParameterId::SERVO_TR_FORM, "servo_tr_form", ParameterType::NATURAL32, true, true},
    {ParameterId::SERVO_TR_VEL, "servo_tr_vel", ParameterType::REAL32, true, true},
}};

consteval bool parameter_catalog_is_valid() {
    for (size_t i = 0; i < PARAMETER_CATALOG.size(); ++i) {
        if (PARAMETER_CATALOG[i].name.empty() || PARAMETER_CATALOG[i].name.size() > 16) {
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

const ParameterDefinition* find_parameter(std::string_view name);
bool read_parameter(const VBDriveConfig& config, ParameterId id, ParameterValue& value);
ParameterWriteResult write_persistent_parameter(
    VBDriveConfig& config,
    ParameterId id,
    const ParameterValue& value,
    bool apply_runtime
);
ParameterWriteResult write_runtime_parameter(ParameterId id, const ParameterValue& value);

void record_invalid_command();
inline bool bootloader_reboot_pending = false;
