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
    hfdcan->Init.ExtFiltersNbr = 4;
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

static constexpr std::string TEST1_PARAM = "test1";
static constexpr std::string TEST2_PARAM = "test2";

void VBDriveConfig::get(std::string& param, UARTResponseAccumulator& responses) {
    if (get_base_params(this, param, responses)) {
        return;
    }
}

bool VBDriveConfig::set(std::string& param, std::string& value, UARTResponseAccumulator& responses) {
    bool was_found = false;
    bool result = set_base_params(this, param, value, responses, was_found);
    if (was_found) {
        return result;
    }

    int new_int_value;
    float new_float_value;
    bool is_converted = false;
    if (param == TEST1_PARAM) {
        is_converted = safe_stoi(value, new_int_value);
    }
    else {
        is_converted = safe_stof(value, new_float_value);
    }
    if (!is_converted) {
        responses.append("ERROR: Invalid value\n\r");
        return false;
    }

    if (param == TEST1_PARAM) {
        test_param1 = new_int_value;
        responses.append("OK: %s:%d\n\r", TEST1_PARAM.c_str(), test_param1);
    }
    else if (param == TEST2_PARAM) {
        test_param2 = new_float_value;
        responses.append("OK: %s:%f\n\r", TEST2_PARAM.c_str(), test_param2);
    }
    else {
        responses.append("ERROR: Unknown parameter\n\r");
        return false;
    }
    return true;
}
