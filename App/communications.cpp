#include "app.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>

#include <voltbro/utils.hpp>

#include <uavcan/diagnostic/Record_1_1.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>

TYPE_ALIAS(DiagnosticRecord, uavcan_diagnostic_Record_1_1)
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;
static std::shared_ptr<CyphalInterface> cyphal_interface;
static bool _is_cyphal_on = false;
static millis delay_cyphal_until_millis = 0;

std::shared_ptr<CyphalInterface> get_interface() {
    return cyphal_interface;
}

void cyphal_error_handler() {
    _is_cyphal_on = false;
    // Clear all queued messages
    cyphal_interface->clear_queue();
    // delay for half a second
    delay_cyphal_until_millis = millis_32() + DELAY_ON_ERROR_MS;
}

void restart_cyphal() {
    // Clear all queued messages, again, just in case
    cyphal_interface->clear_queue();

    static CanardTransferID record_transfer_id = 0;
    DiagnosticRecord::Type record;
    record.severity.value = uavcan_diagnostic_Severity_1_0_ERROR;
    sprintf(reinterpret_cast<char*>(record.text.elements), "cyphal_error_handler was called internally");
    record.text.count = strlen((char*)record.text.elements);

    cyphal_interface->send_msg<DiagnosticRecord>(
            &record,
            uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_,
            &record_transfer_id
    );

    delay_cyphal_until_millis = 0;
    _is_cyphal_on = true;
}

UtilityConfig utilities(micros_64, cyphal_error_handler);

void heartbeat() {
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
        .uptime = (uint32_t)std::floor(millis_32() / 1000.0f),
        .health = {CYPHAL_HEALTH_STATUS},
        .mode = {CYPHAL_MODE},
        .vendor_specific_status_code = static_cast<uint8_t>(cyphal_interface->queue_size())
    };

    if (_is_cyphal_on) {
        cyphal_interface->send_msg<HBeat>(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
        );
    }
}

__attribute__((hot, flatten)) void cyphal_loop() {
    if (_is_cyphal_on) {
        cyphal_interface->loop();
    }
    if (_is_cyphal_on) {
        millis current_t = millis_32();
        in_loop_reporting(current_t);

        static millis heartbeat_time = 0;
        EACH_N(current_t, heartbeat_time, 1000, {
            heartbeat();
        })
    }

    if (delay_cyphal_until_millis != 0 &&
        delay_cyphal_until_millis <= millis_32()) {
        restart_cyphal();
    }
}

void start_cyphal() {
    configure_fdcan(&hfdcan1);

    cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        #ifndef LCM
        get_app_config().get_node_id(),
        #else
        10,
        #endif
        &hfdcan1,
        CYPHAL_QUEUE_SIZE,
        utilities
    ));

    setup_subscriptions();

    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler,
        0
    ))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))
    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))

    _is_cyphal_on = true;
}

void set_cyphal_mode(uint8_t mode) {
    CYPHAL_MODE = mode;
}

__weak void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );
}

__weak void in_loop_reporting(millis current_t) {
    // SEND REGULAR MESSAGES HERE
    // OR DEFINE THIS FUNCTION ELSEWHERE
    // OR MODIFY cyphal_loop
}
