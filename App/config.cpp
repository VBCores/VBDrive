#include "app.h"

#include "usart.h"

AppConfigT app_configurator(
    &huart2,
    get_eeprom(),
    []() {
        auto motor = get_motor();
        if (motor) {
            motor->start();
        }
    },
    []() {
        auto motor = get_motor();
        if (motor) {
            motor->stop();
        }
    }
);

AppConfigT& get_app_config() {
    return app_configurator;
}

void configure_fdcan(FDCAN_HandleTypeDef* hfdcan) {
    hfdcan->Instance = FDCAN1;
    hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV2;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    hfdcan->Init.NominalSyncJumpWidth = 24;
    hfdcan->Init.NominalTimeSeg1 = 55;
    hfdcan->Init.NominalTimeSeg2 = 24;
    hfdcan->Init.DataSyncJumpWidth = 4;
    hfdcan->Init.DataTimeSeg1 = 5;
    hfdcan->Init.DataTimeSeg2 = 4;
    hfdcan->Init.StdFiltersNbr = 0;
    hfdcan->Init.ExtFiltersNbr = 1;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    hfdcan->Init.NominalPrescaler = app_configurator.get_nom_prescaler();
    hfdcan->Init.DataPrescaler = app_configurator.get_data_prescaler();

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    app_configurator.handle_uart_event(huart, size);
}

bool VBDriveConfig::are_required_params_set() {
    return true;
}

static constexpr std::string GEAR_RATIO_PARAM = "gear_ratio";
static constexpr std::string MAX_CURRENT_PARAM = "max_current";
static constexpr std::string MAX_SPEED_PARAM = "max_speed";
static constexpr std::string MAX_TORQUE_PARAM = "max_torque";
static constexpr std::string ANGLE_OFFSET_PARAM = "angle_offset";
static constexpr std::string MIN_ANGLE_PARAM = "min_angle";
static constexpr std::string MAX_ANGLE_PARAM = "max_angle";
static constexpr std::string TORQUE_CONST_PARAM = "torque_const";
static constexpr std::string KP_PARAM = "kp";
static constexpr std::string KI_PARAM = "ki";
static constexpr std::string KD_PARAM = "kd";
static constexpr std::string FILTER_A_PARAM = "filter_a";
static constexpr std::string FILTER_G1_PARAM = "filter_g1";
static constexpr std::string FILTER_G2_PARAM = "filter_g2";
static constexpr std::string FILTER_G3_PARAM = "filter_g3";

void VBDriveConfig::print_self(UARTResponseAccumulator& responses) {
    get(GEAR_RATIO_PARAM, responses);
    get(MAX_CURRENT_PARAM, responses);
    get(MAX_SPEED_PARAM, responses);
    get(MAX_TORQUE_PARAM, responses);
    get(ANGLE_OFFSET_PARAM, responses);
    get(MIN_ANGLE_PARAM, responses);
    get(MAX_ANGLE_PARAM, responses);
    get(TORQUE_CONST_PARAM, responses);
    get(KP_PARAM, responses);
    get(KI_PARAM, responses);
    get(KD_PARAM, responses);
    get(FILTER_A_PARAM, responses);
    get(FILTER_G1_PARAM, responses);
    get(FILTER_G2_PARAM, responses);
    get(FILTER_G3_PARAM, responses);
    get(NODE_ID_PARAM, responses);
    get(FDCAN_DATA_PARAM, responses);
    get(FDCAN_NOMINAL_PARAM, responses);
    responses.append("was configured:%s\n\r", was_configured ? "true" : "false");
    responses.append("are all required params set: %s\n\r", are_required_params_set() ? "true" : "false");
}

void VBDriveConfig::get(const std::string& param, UARTResponseAccumulator& responses) {
    if (get_base_params(this, param, responses)) {
        return;
    }
    CHECK_AND_PRINT_PARAM_INT(static_cast<int>(gear_ratio), GEAR_RATIO_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(max_current, MAX_CURRENT_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(max_speed, MAX_SPEED_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(max_torque, MAX_TORQUE_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(angle_offset, ANGLE_OFFSET_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(min_angle, MIN_ANGLE_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(max_angle, MAX_ANGLE_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(torque_const, TORQUE_CONST_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(ki, KI_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(kp, KP_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(kd, KD_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(filter_a, FILTER_A_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(filter_g1, FILTER_G1_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(filter_g2, FILTER_G2_PARAM)
    CHECK_AND_PRINT_PARAM_FLOAT(filter_g3, FILTER_G3_PARAM)
    else {
        responses.append("ERROR: Unknown parameter\n\r");
    }
}

bool VBDriveConfig::set(const std::string& param, std::string& value, UARTResponseAccumulator& responses) {
    bool was_found = false;
    bool result = set_base_params(this, param, value, responses, was_found);
    if (was_found) {
        return result;
    }

    int new_int_value;
    float new_float_value;
    bool is_converted = false;
    if (param == GEAR_RATIO_PARAM) {
        is_converted = safe_stoi(value, new_int_value);
    } else {
        is_converted = safe_stof(value, new_float_value);
    }
    if (!is_converted) {
        responses.append("ERROR: Invalid value\n\r");
        return false;
    }

    if (param == GEAR_RATIO_PARAM) {
        gear_ratio = static_cast<uint8_t>(new_int_value);
        responses.append("OK: gear_ration :%d\n\r", static_cast<int>(gear_ratio));
    }
    CHECK_AND_SET_PARAM_FLOAT(max_current, MAX_CURRENT_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(max_speed, MAX_SPEED_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(max_torque, MAX_TORQUE_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(angle_offset, ANGLE_OFFSET_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(min_angle, MIN_ANGLE_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(max_angle, MAX_ANGLE_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(torque_const, TORQUE_CONST_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(ki, KI_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(kp, KP_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(kd, KD_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(filter_a, FILTER_A_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(filter_g1, FILTER_G1_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(filter_g2, FILTER_G2_PARAM)
    CHECK_AND_SET_PARAM_FLOAT(filter_g3, FILTER_G3_PARAM)
    else {
        responses.append("ERROR: Unknown parameter\n\r");
        return false;
    }
    return true;
}
