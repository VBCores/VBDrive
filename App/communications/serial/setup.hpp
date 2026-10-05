#pragma once

#include "app.h"
#include "interface.hpp"
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

char serial_tx_buffer[UART_TX_BUFFER_SIZE];
void start_uart_recv_it();

static constexpr uint16_t UART_RX_BUFFER_SIZE = 32;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
// DMA producer, bounded main-loop consumer; indices are accessed with IRQs masked.
static char serial_fifo[512];
static size_t serial_head = 0, serial_tail = 0;
static bool serial_overflow = false, serial_discarding = false;
static char serial_line[192];
static size_t serial_length = 0;
static bool serial_line_discarding = false;
static volatile bool serial_enabled = false, serial_busy = false;
volatile bool serial_deferred = false;

extern "C" void serial_service();

// Blocking commands run in thread mode while normal motor updates are paused.
void process_serial() {
    serial_enabled = true;
    static millis service_time = 0;
    const millis now = millis_32();
    if (!serial_deferred && now != service_time) {
        service_time = now;
        serial_service();
    }
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
            serial_send_message("SERIAL ERROR: input overflow\r\n");
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
                const auto param = split_parameter(command);
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
                serial_send_message("SERIAL ERROR: Invalid or overlong command\r\n");
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
            serial_send_message("state: %.6f %.6f %.6f\r\n",
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

