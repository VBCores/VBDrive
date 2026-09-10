#include "app.h"

#include "usart.h"

#include <cmath>

static constexpr size_t PARSED_VALUE_MAX_SIZE = 31;

void VBDriveConfig::apply_servo_config() const {
    auto motor = get_motor();
    if (!motor) return;
    motor->update_servo_config(SetPointType::POSITION,
        PIDConfig{.kp = value_or_default(servo_pos_p_gain, VBDriveDefaults::SERVO_POS_P_GAIN),
                  .ki = value_or_default(servo_pos_i_gain, VBDriveDefaults::SERVO_POS_I_GAIN),
                  .kd = value_or_default(servo_pos_d_gain, VBDriveDefaults::SERVO_POS_D_GAIN)});
    motor->update_servo_config(SetPointType::VELOCITY,
        PIDConfig{.kp = value_or_default(servo_vel_p_gain, VBDriveDefaults::SERVO_VEL_P_GAIN),
                  .ki = value_or_default(servo_vel_i_gain, VBDriveDefaults::SERVO_VEL_I_GAIN)});
}

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
// DMA producer, bounded PendSV consumer; indices are accessed with IRQs masked.
static char serial_fifo[512];
static size_t serial_head = 0, serial_tail = 0;
static bool serial_overflow = false, serial_discarding = false;
static char serial_line[192];
static size_t serial_length = 0;
static bool serial_line_discarding = false;
static volatile bool serial_enabled = false, serial_busy = false;
volatile bool serial_deferred = false;

void serial_tick() {
    if (serial_enabled && !serial_busy && !serial_deferred &&
        get_app_manager().get_state() != CommandState::CALIBRATING) {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

// Blocking commands run in thread mode while normal motor updates are paused.
void process_serial() {
    serial_enabled = true;
    if (!serial_deferred) return;
    serial_busy = true;
    if (std::string_view(serial_line, serial_length) == "APPLY") {
        if (auto motor = get_motor()) motor->stop();
    }
    get_app_manager().process_command(std::string_view(serial_line, serial_length));
    reboot_to_bootloader_if_requested();
    serial_length = 0;
    serial_deferred = false;
    serial_busy = false;
}

void discard_serial_input() {
    CRITICAL_SECTION({
        serial_tail = serial_head;
        serial_overflow = false;
        serial_discarding = false;
        serial_length = 0;
        serial_line_discarding = false;
    })
}

static void receive_serial(std::string_view bytes) {
    // Calibration is isolated: never defer its input until motion resumes.
    if (get_app_manager().get_state() == CommandState::CALIBRATING) {
        serial_tail = serial_head;
        serial_overflow = false;
        serial_discarding = bytes.empty() || (bytes.back() != '\r' && bytes.back() != '\n');
        return;
    }
    for (char byte : bytes) {
        if (serial_discarding) {
            if (byte != '\r' && byte != '\n') continue;
            serial_discarding = false;
        }
        const size_t next = (serial_head + 1) % sizeof(serial_fifo);
        if (next == serial_tail) {
            serial_tail = serial_head;
            serial_overflow = true;
            serial_discarding = byte != '\r' && byte != '\n';
            continue;
        }
        serial_fifo[serial_head] = byte;
        serial_head = next;
    }
}

extern "C" void serial_service() {
    auto& line = serial_line;
    auto& length = serial_length;
    auto& discarding = serial_line_discarding;
    auto& manager = get_app_manager();
    if (!serial_enabled || serial_busy || serial_deferred ||
        manager.get_state() == CommandState::CALIBRATING ||
        huart2.gState != HAL_UART_STATE_READY) return;
    for (;;) {
        char byte = 0;
        bool lost = false, present = false;
        CRITICAL_SECTION({
            lost = serial_overflow;
            serial_overflow = false;
            present = !lost && serial_tail != serial_head;
            if (present) {
                byte = serial_fifo[serial_tail];
                serial_tail = (serial_tail + 1) % sizeof(serial_fifo);
            }
        })
        if (lost) {
            length = 0;
            discarding = false; // The ISR already discarded through the next delimiter.
            manager.send_message("ERROR: Serial input overflow\r\n");
            return;
        }
        if (!present) break;
        if (byte == '\r' || byte == '\n') {
            if (!discarding && length != 0) {
                const size_t command_length = length;
                line[command_length] = '\0';
                auto command = std::string_view(line, command_length);
                const auto end = command.find_last_not_of(" \t");
                command = command.substr(0, end == std::string_view::npos ? 0 : end + 1);
                const auto param = manager.split_parameter(command);
                if (command == "INFO" || command == "HELP" ||
                    command == "CONFIG" || command == "EXIT" || command == "SAVE" ||
                    command == "APPLY" || command == "CALIBRATE" ||
                    (param && std::get<1>(*param) != "?" &&
                     (std::get<0>(*param) == "is_on" || std::get<0>(*param) == "bootloader"))) {
                    // Driver I2C, EEPROM, calibration and reset must not run in an ISR.
                    length = command.size();
                    serial_deferred = true;
                    return;
                }
                length = 0;
                manager.process_command(command);
                return; // At most one command per tick; never wait for UART here.
            } else {
                length = 0;
                discarding = false;
            }
        } else if (!discarding) {
            if (length == sizeof(line) - 1 || byte == '\0') {
                length = 0;
                discarding = true;
                manager.send_message("ERROR: Invalid or overlong command\r\n");
                return;
            } else line[length++] = byte;
        }
    }
    static millis logging_time = 0;
    const millis now = millis_32();
    if (now - logging_time >= 10 && manager.is_logging()) {
        // Keep the 100 Hz grid despite TX contention; never queue stale state samples.
        logging_time = now - (now - logging_time) % 10;
        if (auto motor = get_motor()) {
            manager.send_message("state: %.6f %.6f %.6f\r\n",
                                 motor->get_angle(), motor->get_velocity(), motor->get_torque());
        }
    }
}

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
    receive_serial(std::string_view(uart_rx_buffer, size));
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
