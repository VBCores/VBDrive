#include "app.h"

FDCAN_HandleTypeDef hfdcan1;

extern "C" {

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    if(fdcanHandle->Instance==FDCAN1) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle) {
    if(fdcanHandle->Instance==FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
}

}
