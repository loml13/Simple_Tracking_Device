/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */
/* 引用串口句柄 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/**
 * @brief UART3普通发送函数
 * @param data: 要发送的数据
 * @param size: 数据大小
 * @param timeout: 超时时间
 * @return HAL状态
 */
HAL_StatusTypeDef UART3_Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_UART_Transmit(&huart3, data, size, timeout);
}

/**
 * @brief UART6普通发送函数
 * @param data: 要发送的数据
 * @param size: 数据大小
 * @param timeout: 超时时间
 * @return HAL状态
 */
HAL_StatusTypeDef UART6_Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_UART_Transmit(&huart6, data, size, timeout);
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

