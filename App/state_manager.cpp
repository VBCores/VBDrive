#include "app.h"

#include "usart.h"

#include <cmath>

constexpr auto make_action(auto f1, auto f2) {
    return std::make_tuple(
        std::function<bool()>(f1),
        std::function<bool()>(f2)
    );
}

DriveStateController drive_state_controller(
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
    },
    DriveStateController::ActionsMap{
        { "CALIBRATE", make_action(is_able_to_calibrate, do_calibrate)}
    }
);

DriveStateController& get_app_manager() {
    return drive_state_controller;
}

static constexpr uint16_t UART_RX_BUFFER_SIZE = 32;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE + 1];

void start_uart_recv_it() {
    std::memset(uart_rx_buffer, '\0', UART_RX_BUFFER_SIZE + 1);
    /*
    / If TX uses DMA, next line is not needed,
    / if TX is IT or direct, next line is required due to bug in HAL.
    / Kept here so I don't forget
    HAL_UART_Abort_IT(&huart2);
    */
    auto status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2,
        reinterpret_cast<uint8_t*>(uart_rx_buffer),
        UART_RX_BUFFER_SIZE
    );
    if (status != HAL_OK) {
        // TODO
    }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
// NOTE: huart parameter required by the interface, but not used in this implementation
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
#pragma GCC diagnostic pop
    uart_rx_buffer[size] = '\0';
    auto command_string = std::string(uart_rx_buffer);

    drive_state_controller.process_command(command_string);
    drive_state_controller.wait_for_uart();
    start_uart_recv_it();
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
    hfdcan->Init.ExtFiltersNbr = 4;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    hfdcan->Init.NominalPrescaler = drive_state_controller.get_nom_prescaler();
    hfdcan->Init.DataPrescaler = drive_state_controller.get_data_prescaler();

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        Error_Handler();
    }
}

bool VBDriveConfig::are_required_params_set() {
    return BaseConfigData::are_required_params_set() && gear_ratio != 0;
}

static constexpr std::string GEAR_RATIO_PARAM = "gear";
static constexpr std::string MAX_CURRENT_PARAM = "max_i";
static constexpr std::string MAX_SPEED_PARAM = "max_spd";
static constexpr std::string MAX_TORQUE_PARAM = "max_tq";
static constexpr std::string ANGLE_OFFSET_PARAM = "ang_off";
static constexpr std::string MIN_ANGLE_PARAM = "min_ang";
static constexpr std::string MAX_ANGLE_PARAM = "max_ang";
static constexpr std::string TORQUE_CONST_PARAM = "kt";
static constexpr std::string KP_PARAM = "kp";
static constexpr std::string KI_PARAM = "ki";
static constexpr std::string KD_PARAM = "kd";
static constexpr std::string FILTER_A_PARAM = "flt_a";
static constexpr std::string FILTER_G1_PARAM = "flt_g1";
static constexpr std::string FILTER_G2_PARAM = "flt_g2";
static constexpr std::string FILTER_G3_PARAM = "flt_g3";
static constexpr std::string FILTER_I_LPF_PARAM = "i_lpf";
static constexpr std::string ANGLE_ENCODER_PARAM = "ang_enc";

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
    get(FILTER_I_LPF_PARAM, responses);
    get(NODE_ID_PARAM, responses);
    get(FDCAN_DATA_PARAM, responses);
    get(FDCAN_NOMINAL_PARAM, responses);
    get(ANGLE_ENCODER_PARAM, responses);
    responses.append("was configured:%s\n\r", was_configured ? "true" : "false");
    responses.append("are all required params set: %s\n\r", are_required_params_set() ? "true" : "false");
}

void VBDriveConfig::get(const std::string& param, UARTResponseAccumulator& responses) {
    if (get_base_params(this, param, responses)) {
        return;
    }
    if (param == GEAR_RATIO_PARAM) {
        responses.append("gear:%u\n\r", value_or_default(gear_ratio, VBDriveDefaults::GEAR_RATIO, static_cast<uint8_t>(0)));
    }
    else if (param == MAX_CURRENT_PARAM) {
        responses.append("max_i:%f\n\r", value_or_default(max_current, NAN));
    }
    else if (param == MAX_SPEED_PARAM) {
        responses.append("max_spd:%f\n\r", value_or_default(max_speed, NAN));
    }
    else if (param == MAX_TORQUE_PARAM) {
        responses.append("max_tq:%f\n\r", value_or_default(max_torque, NAN));
    }
    else if (param == ANGLE_OFFSET_PARAM) {
        responses.append("ang_off:%f\n\r", value_or_default(angle_offset, VBDriveDefaults::ANGLE_OFFSET));
    }
    else if (param == MIN_ANGLE_PARAM) {
        responses.append("min_ang:%f\n\r", value_or_default(min_angle, NAN));
    }
    else if (param == MAX_ANGLE_PARAM) {
        responses.append("max_ang:%f\n\r", value_or_default(max_angle, NAN));
    }
    else if (param == TORQUE_CONST_PARAM) {
        responses.append("kt:%f\n\r", value_or_default(torque_const, VBDriveDefaults::TORQUE_CONST));
    }
    else if (param == KI_PARAM) {
        responses.append("ki:%f\n\r", value_or_default(ki, VBDriveDefaults::PID_KI));
    }
    else if (param == KP_PARAM) {
        responses.append("kp:%f\n\r", value_or_default(kp, VBDriveDefaults::PID_KP));
    }
    else if (param == KD_PARAM) {
        responses.append("kd:%f\n\r", value_or_default(kd, VBDriveDefaults::PID_KD));
    }
    else if (param == FILTER_A_PARAM) {
        responses.append("flt_a:%f\n\r", value_or_default(filter_a, VBDriveDefaults::FILTER_A));
    }
    else if (param == FILTER_G1_PARAM) {
        responses.append("flt_g1:%f\n\r", value_or_default(filter_g1, VBDriveDefaults::FILTER_G1));
    }
    else if (param == FILTER_G2_PARAM) {
        responses.append("flt_g2:%f\n\r", value_or_default(filter_g2, VBDriveDefaults::FILTER_G2));
    }
    else if (param == FILTER_G3_PARAM) {
        responses.append("flt_g3:%f\n\r", value_or_default(filter_g3, VBDriveDefaults::FILTER_G3));
    }
    else if (param == FILTER_I_LPF_PARAM) {
        responses.append("i_lpf:%f\n\r", value_or_default(I_lpf_coefficient, VBDriveDefaults::I_LPF));
    }
    else if (param == ANGLE_ENCODER_PARAM) {
        responses.append("ang_enc:%u\n\r", to_underlying(angle_encoder));
    }
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

    int new_int_value = 0;
    float new_float_value = 0;
    bool is_converted = false;
    if (param == GEAR_RATIO_PARAM || param == ANGLE_ENCODER_PARAM) {
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
        responses.append("OK: gear:%d\n\r", static_cast<int>(gear_ratio));
    }
    else if (param == ANGLE_ENCODER_PARAM) {
        angle_encoder = static_cast<AngleEncoderType>(new_int_value);
        responses.append("OK: ang_enc:%u\n\r", to_underlying(angle_encoder));
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
    CHECK_AND_SET_PARAM_FLOAT(I_lpf_coefficient, FILTER_I_LPF_PARAM)
    else {
        responses.append("ERROR: Unknown parameter\n\r");
        return false;
    }
    return true;
}
