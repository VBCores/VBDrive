#include "app.h"

#include "usart.h"

#include <cmath>

static constexpr size_t PARSED_VALUE_MAX_SIZE = 31;

bool parse_serial_number(std::string_view str, int& out_val) {
    if (str.empty()) return false;
    if (str.size() > PARSED_VALUE_MAX_SIZE) return false;

    char buffer[PARSED_VALUE_MAX_SIZE + 1];
    memcpy(buffer, str.data(), str.size());
    buffer[str.size()] = '\0';

    char* endptr = nullptr;
    long val = strtol(buffer, &endptr, 10);

    // Check for conversion errors
    if (endptr != buffer + str.size()) return false;
    if (val < INT32_MIN || val > INT32_MAX) return false;

    out_val = static_cast<int>(val);
    return true;
}

bool parse_serial_number(std::string_view str, float& out_val) {
    if (str.empty()) return false;
    if (str.size() > PARSED_VALUE_MAX_SIZE) return false;

    char buffer[PARSED_VALUE_MAX_SIZE + 1];
    memcpy(buffer, str.data(), str.size());
    buffer[str.size()] = '\0';

    char* endptr = nullptr;
    float val = strtof(buffer, &endptr);

    if (endptr != buffer + str.size()) return false;
    if (val == HUGE_VALF || val == -HUGE_VALF) return false;

    out_val = val;
    return true;
}


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

#define BOOT_REQUEST_MAGIC 0xB00710ADUL
static void boot_request_access_enable() {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

static void boot_request_access_disable() {
    HAL_PWR_DisableBkUpAccess();
}

void reboot_to_bootloader() {
    auto motor = get_motor();

    if (motor != nullptr) {
        motor->set_foc_point(FOCTarget{0});
        motor->stop();
    }

    (void)HAL_FDCAN_Stop(&hfdcan1);

    boot_request_access_enable();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    TAMP->BKP1R = 0U;
    TAMP->BKP2R = 0U;
    boot_request_access_disable();

    HAL_Delay(50);
    NVIC_SystemReset();
}

static constexpr uint16_t UART_RX_BUFFER_SIZE = 32;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];

void start_uart_recv_it() {
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
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
// NOTE: huart parameter required by the interface, but not used in this implementation
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
#pragma GCC diagnostic pop
    drive_state_controller.process_command(std::string_view(uart_rx_buffer, size));
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

void VBDriveConfig::print_self(UARTResponseAccumulator& responses) {
    for (const auto& definition : PARAMETER_CATALOG) {
        if (definition.is_persistent) {
            get(definition.name, responses);
        }
    }
    responses.append("was configured:%s\n\r", was_configured ? "true" : "false");
    responses.append("are all required params set: %s\n\r", are_required_params_set() ? "true" : "false");
}

void VBDriveConfig::get(std::string_view param, UARTResponseAccumulator& responses) {
    const auto* definition = find_parameter(param);
    if (!definition) {
        responses.append("ERROR: Unknown parameter\n\r");
        return;
    }
    ParameterValue value{};
    if (!read_parameter(*this, definition->id, value)) {
        responses.append("ERROR: Parameter unavailable\n\r");
        return;
    }
    switch (definition->type) {
        case ParameterType::REAL32:
            responses.append("%s:%f\n\r", definition->name.data(), std::get<float>(value));
            break;
        case ParameterType::NATURAL32:
            responses.append("%s:%lu\n\r", definition->name.data(), std::get<uint32_t>(value));
            break;
        case ParameterType::INTEGER32:
            responses.append("%s:%ld\n\r", definition->name.data(), std::get<int32_t>(value));
            break;
        case ParameterType::BIT:
            responses.append("%s:%u\n\r", definition->name.data(), std::get<bool>(value) ? 1U : 0U);
            break;
        case ParameterType::STRING:
            responses.append("%s:%.*s\n\r", definition->name.data(), static_cast<int>(std::get<std::string_view>(value).size()), std::get<std::string_view>(value).data());
            break;
    }
}

bool VBDriveConfig::set(std::string_view param, std::string_view input, UARTResponseAccumulator& responses, bool apply_runtime) {
    const auto* definition = find_parameter(param);
    if (!definition) {
        responses.append("ERROR: Unknown parameter\n\r");
        return false;
    }
    if (!definition->is_persistent) {
        responses.append(definition->is_mutable
            ? "ERROR: Parameter unavailable\n\r"
            : "ERROR: Read-only parameter\n\r");
        return false;
    }

    ParameterValue value{};
    int integer = 0;
    if (definition->type == ParameterType::REAL32) {
        if (!parse_serial_number(input, value.emplace<float>())) {
            responses.append("ERROR: Invalid value\n\r");
            return false;
        }
    }
    else if (definition->type == ParameterType::NATURAL32 || definition->type == ParameterType::INTEGER32) {
        if (!parse_serial_number(input, integer) || (definition->type == ParameterType::NATURAL32 && integer < 0)) {
            responses.append("ERROR: Invalid value\n\r");
            return false;
        }
        if (definition->type == ParameterType::INTEGER32) value = static_cast<int32_t>(integer);
        else value = static_cast<uint32_t>(integer);
    }
    else {
        responses.append("ERROR: Invalid value\n\r");
        return false;
    }

    if (write_persistent_parameter(*this, definition->id, value, apply_runtime) != ParameterWriteResult::OK) {
        responses.append("ERROR: Invalid value\n\r");
        return false;
    }
    if (definition->type == ParameterType::REAL32) {
        responses.append("OK: %s:%f\n\r", definition->name.data(), std::get<float>(value));
    }
    else if (definition->type == ParameterType::INTEGER32) {
        responses.append("OK: %s:%ld\n\r", definition->name.data(), std::get<int32_t>(value));
    }
    else {
        responses.append("OK: %s:%lu\n\r", definition->name.data(), std::get<uint32_t>(value));
    }
    return true;
}
