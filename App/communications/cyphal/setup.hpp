#pragma once

#include "app.h"
#include "state_manager/state_manager.h"
#include "profiling.hpp"
#include <voltbro/motors/bldc/foc/foc.hpp>
#include <cyphal/cyphal.h>
#include <cyphal/providers/G4CAN.h>
#include <cyphal/allocators/o1/o1_allocator.h>
#include <voltbro/utils.hpp>
#include <uavcan/diagnostic/Record_1_1.hpp>
#include <uavcan/node/Heartbeat_1_0.hpp>
#include <uavcan/node/Health_1_0.hpp>
#include <uavcan/node/Mode_1_0.hpp>

inline constexpr size_t CYPHAL_QUEUE_SIZE = 50;
inline constexpr size_t SHARED_BUFFER_SIZE = std::max(
    (CALIBRATION_BUFF_SIZE + 1) * sizeof(int),
    static_cast<size_t>(CYPHAL_QUEUE_SIZE * sizeof(CanardTxQueueItem) * QUEUE_SIZE_MULT));
inline constexpr millis DELAY_ON_ERROR_MS = 500;
void setup_subscriptions();
void in_loop_reporting(millis);

FDCAN_HandleTypeDef hfdcan1;

extern "C" {

#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
void FDCAN1_IT0_IRQHandler() { HAL_FDCAN_IRQHandler(&hfdcan1); }
#endif

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    if (fdcanHandle->Instance == FDCAN1) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle) {
    if (fdcanHandle->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
}

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

    hfdcan->Init.NominalPrescaler = voltbro_can_nominal_prescaler(get_app_manager().get_committed_config().base.fdcan_nominal_baud);
    hfdcan->Init.DataPrescaler = voltbro_can_data_prescaler(get_app_manager().get_committed_config().base.fdcan_data_baud);

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        Error_Handler();
    }
}

using DiagnosticRecord = uavcan_diagnostic_Record_1_1;
using HBeat = uavcan_node_Heartbeat_1_0;

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
    DiagnosticRecord record;
    record.severity.value = uavcan_diagnostic_Severity_1_0_ERROR;
    sprintf(reinterpret_cast<char*>(record.text.elements), "cyphal_error_handler was called internally");
    record.text.count = strlen((char*)record.text.elements);

    cyphal_interface->send_msg(
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
    HBeat heartbeat_msg = {
        .uptime = (uint32_t)std::floor(millis_32() / 1000.0f),
        .health = {CYPHAL_HEALTH_STATUS},
        .mode = {CYPHAL_MODE},
        .vendor_specific_status_code = static_cast<uint8_t>(cyphal_interface->queue_size())
    };

    if (_is_cyphal_on) {
        cyphal_interface->send_msg(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
        );
    }
}

__attribute__((hot)) void cyphal_loop() {
    VBDRIVE_PROFILE_COUNT(cyphal_loop_invocations)
    if (_is_cyphal_on) {
        cyphal_interface->process_canard_rx(false);
    }
    if (_is_cyphal_on) {
        millis current_t = millis_32();
        in_loop_reporting(current_t);
        cyphal_interface->process_canard_tx(false);

        static millis heartbeat_time = 0;
        EACH_N(current_t, heartbeat_time, 1000, {
            heartbeat();
        })
    }

    if (delay_cyphal_until_millis != 0 &&
        delay_cyphal_until_millis <= millis_32()) {
        restart_cyphal();
    }
    if (_is_cyphal_on) {
        cyphal_interface->update_fdcan_status();
    }
}

static std::byte cyphal_bss_buffer[
    sizeof(CyphalInterface) +
    sizeof(G4CAN) +
    sizeof(O1Allocator)
] __attribute__((aligned(4)));
std::byte cyphal_queue_buffer_shared[SHARED_BUFFER_SIZE] __attribute__((aligned(O1HEAP_ALIGNMENT)));

void start_cyphal() {
    configure_fdcan(&hfdcan1);

    cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_bss<G4CAN, O1Allocator>(
        cyphal_bss_buffer,
        get_app_manager().get_node_id(),
        &hfdcan1,
        CYPHAL_QUEUE_SIZE,
        utilities,
        cyphal_queue_buffer_shared
    ));

    setup_subscriptions();

    // When a burst fills the three-element hardware FIFO, keep the newest
    // command instead of the oldest unprocessed one.
    HAL_IMPORTANT(HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE))

    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler,
        0
    ))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))
    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))
#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
    HAL_IMPORTANT(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
#endif

    _is_cyphal_on = true;
}

void set_cyphal_mode(uint8_t mode) {
    CYPHAL_MODE = mode;
}

#if defined(FOC_PROFILE) || defined(CYPHAL_PROFILE)
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t flags) {
    if (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) VBDRIVE_PROFILE_HANDLER(can_arrival_timing);
}
#endif

// Calibration temporarily owns the arena; subscription descriptors stay in BSS.
void pause_cyphal_for_calibration() {
    _is_cyphal_on = false;
    delay_cyphal_until_millis = 0;
    HAL_IMPORTANT(HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2))
    HAL_IMPORTANT(HAL_FDCAN_Stop(&hfdcan1))
    cyphal_interface->halt();
}

void resume_cyphal_after_calibration() {
    cyphal_interface->reset();
    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))
    const auto pending = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    for (uint32_t i = 0; i < pending; ++i) {
        FDCAN_RxHeaderTypeDef header{};
        uint8_t data[64];
        HAL_IMPORTANT(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data))
    }
    _is_cyphal_on = true;
}
