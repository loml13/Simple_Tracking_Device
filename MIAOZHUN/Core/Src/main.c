/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DATOU.h"  /* 云台控制相关函数 */
#include <string.h> /* for strlen */
#include <stdio.h>  /* for sprintf */
#include "frame.h" /* 帧解析相关函数 */
#include "Key.h"   /* 按键处理函数 */
#include "dma.h"   /* DMA相关函数 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Key2_Motor_Control_Process(void);
void Key3_Motor_Control_Process(void);
void Test_Key2_Process_Demo(void);
/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aRxBuffer[2]; // 用于UART3和UART6接收的缓冲区

// 串口4接收缓冲区
uint8_t uart4_rx_buffer[1];
volatile uint8_t uart4_data_received = 0;
volatile uint8_t uart4_received_data = 0;

static  uint8_t turn_z[13] = {                                                
        0x01,       // [0] 地址
        0xFD,       // [1] 功能码（位置模式）
        0x01,       // [2] 方向（0x00=CW, 0x01=CCW）
        0x00, 0x10, // [3-4] 速度 30 RPM
        0x00,       // [5] 加速度
        // [6..9] 脉冲数占位：
        0x00, 0x00, 0x0C, 0x80,
        0x00,       // [10] 相对/绝对 模式（0=相对,1=绝对）
        0x00,       // [11] 多机同步 标志
        0x6B        // [12] 校验字（固定）
    };
static uint8_t turn_f[13] = {
        0x01,       // [0] 地址
        0xFD,       // [1] 功能码（位置模式）
        0x00,       // [2] 方向（0x00=CW, 0x01=CCW）
        0x00, 0x10, // [3-4] 速度 30 RPM
        0x00,       // [5] 加速度
        // [6..9] 脉冲数占位：
        0x00, 0x00, 0x0C, 0x80,
        0x0,       // [10] 相对/绝对 模式（0=相对,1=绝对）
        0x00,       // [11] 多机同步 标志
        0x6B        // [12] 校验字（固定）
    };

  static  uint8_t enable[]={0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B};



/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // 初始化串口接收中断
  HAL_UART_Receive_IT(&huart3, &aRxBuffer[0], 1);
  HAL_UART_Receive_IT(&huart6, &aRxBuffer[1], 1);

    //zero_param_x(); // 初始化云台参数
    //zero_param_y(); // 初始化云台参数
    //set_zero_pos_x(); // 设置云台X轴零位
    //set_zero_pos_y(); // 设置云台Y轴零位
    //stop()  ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(Key_getnum() == 1) // 按键1被按下 - 测试功能
    {
        Remove();
    }
    
    if(Key_getnum() == 2) // 按键2被按下
    {
        Key2_Motor_Control_Process();
    }
    if(Key_getnum() == 3)
    {
      Key3_Motor_Control_Process();
    }
    if(Key_getnum() == 4)
    {
      UART3_Transmit(enable, sizeof(enable), 100);
      HAL_Delay(100); // 确保命令发送完成
      UART3_Transmit(turn_z, sizeof(turn_z), 100);

    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 按键2检测后的电机控制处理函数
 */
void Key2_Motor_Control_Process(void)
{
    // 1. 检查串口3状态，发送电机正转命令

        uint8_t wakeup_cmd[] = {0xA1};
        HAL_UART_Transmit(&huart4, wakeup_cmd, sizeof(wakeup_cmd), 100);
       UART3_Transmit(enable, sizeof(enable), 100);
       HAL_Delay(100); // 确保命令发送完成
       UART3_Transmit(turn_f, sizeof(turn_f), 100);

    
    // 3. 进入循环，等待UART4接收数据并处理
    while(1) 
    {
        // 启动串口4接收，等待数据
        uart4_data_received = 0;
        if(HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) 
        {
            HAL_UART_Receive_IT(&huart4, uart4_rx_buffer, 1);
            
            char wait_msg[] = "Waiting for UART4 data...\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)wait_msg, strlen(wait_msg), 100);
        }
        
        // 4. while循环等待uart4接收到数据
        while(!uart4_data_received) 
        {
            // 等待接收数据，可以添加超时退出机制
            HAL_Delay(1);
        }
        
        // 5. 判断接收到的数据，并用串口3发送相应数据
        char recv_msg[50];
        sprintf(recv_msg, "Received data: 0x%02X ('%c')\r\n", uart4_received_data, uart4_received_data);
        HAL_UART_Transmit(&huart1, (uint8_t*)recv_msg, strlen(recv_msg), 100);

            // 根据接收到的数据判断并发送相应命令
            switch(uart4_received_data) 
            {
                case 0x88:
                    {
                        uint8_t action_msg[] = {0x01, 0xFD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x80, 0x00, 0x00, 0x6B};
                        char debug_msg[] = "Received 0x88, sending slow forward command...\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
                        HAL_UART_Transmit(&huart3, enable, sizeof(enable), 100);
                        HAL_Delay(20);
                        HAL_UART_Transmit(&huart3, action_msg, 13, 100);
                        
                        HAL_Delay(52); // 确保命令发送完成
                        stop(); // 停止电机
                        HAL_Delay(200);  // 确保停止命令完成
                        
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // 停止电机
                        
                    }
                    break;
                    
                default:
                    {
                        char unknown_msg[] = "Unknown command, no action taken\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)unknown_msg, strlen(unknown_msg), 100);
                    }
                    break;
            }
        }
    
    }


/**
 * @brief 按键3检测后的电机控制处理函数
 */
void Key3_Motor_Control_Process(void)
{
    // 1. 检查串口3状态，发送电机反转命令
     UART3_Transmit(enable, sizeof(enable), 100);
       HAL_Delay(100); // 确保命令发送完成
       UART3_Transmit(turn_z, sizeof(turn_z), 100);

        
    
    // 2. 用串口4发送数据
    if(HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) 
    {
        uint8_t wakeup_cmd[] = {0xA1};
        HAL_UART_Transmit(&huart4, wakeup_cmd, sizeof(wakeup_cmd), 100);
    }
    
    // 3. 进入循环，等待UART4接收数据并处理
    while(1) 
    {
        // 启动串口4接收，等待数据
        uart4_data_received = 0;
        if(HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) 
        {
            HAL_UART_Receive_IT(&huart4, uart4_rx_buffer, 1);
            
            char wait_msg[] = "Waiting for UART4 data...\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)wait_msg, strlen(wait_msg), 100);
        }
        
        // 4. while循环等待uart4接收到数据
        while(!uart4_data_received) 
        {
            // 等待接收数据，可以添加超时退出机制
            HAL_Delay(1);
        }
        
        // 5. 判断接收到的数据，并用串口3发送相应数据
        char recv_msg[50];
        sprintf(recv_msg, "Received data: 0x%02X ('%c')\r\n", uart4_received_data, uart4_received_data);
        HAL_UART_Transmit(&huart1, (uint8_t*)recv_msg, strlen(recv_msg), 100);

        // 根据接收到的数据判断并发送相应命令
        switch(uart4_received_data) 
        {
            case 0x88:
                {
                    uint8_t action_msg[] = {0x01, 0xFD, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x80, 0x00, 0x00, 0x6B};
                        char debug_msg[] = "Received 0x88, sending slow forward command...\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
                        HAL_UART_Transmit(&huart3, enable, sizeof(enable), 100);
                        HAL_Delay(20);
                        HAL_UART_Transmit(&huart3, action_msg, 13, 100);
                        
                        HAL_Delay(52); // 确保命令发送完成
                        stop(); // 停止电机
                        HAL_Delay(200);  // 确保停止命令完成
                        
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // 停止电机
                }
                break;

                
            default:
                {
                    char unknown_msg[] = "Unknown command, no action taken\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)unknown_msg, strlen(unknown_msg), 100);
                }
                break;
        }
        
        // 6. 短暂延时后继续下一轮循环
        HAL_Delay(100);
    }
}

/**
 * @brief 测试演示函数 - 模拟整个流程
 */
void Test_Key2_Process_Demo(void)
{
    char demo_msg[] = "=== Key2 Process Demo ===\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)demo_msg, strlen(demo_msg), 100);
    
    // 模拟按键2按下的完整流程
    Key2_Motor_Control_Process();
}

/**
 * @brief 串口接收完成回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 发送调试信息
    if(huart->Instance == USART2) {
        // 不处理调试串口的回调，避免干扰调试消息输出
        return;
    }
    
    char debug_msg[50];
    sprintf(debug_msg, "HAL_UART_RxCpltCallback triggered for UART%d\r\n", 
            (huart->Instance == USART1) ? 1 : 
            (huart->Instance == USART3) ? 3 : 
            (huart->Instance == USART6) ? 6 : 
            (huart->Instance == UART4) ? 4 : 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    if(huart->Instance == UART4) { // 串口4
        uart4_received_data = uart4_rx_buffer[0];
        uart4_data_received = 1;
    }
    else if(huart->Instance == USART3 || huart->Instance == USART6) { 
        // 使用普通串口接收
        HAL_UART_Receive_IT(huart, &aRxBuffer[huart->Instance == USART3 ? 0 : 1], 1);
    }
}

/**
 * @brief 串口发送完成回调函数
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 发送调试信息
    if(huart->Instance == USART2) {
        // 不处理调试串口的回调，避免干扰调试消息输出
        return;
    }
    
    char debug_msg[50];
    sprintf(debug_msg, "HAL_UART_TxCpltCallback triggered for UART%d\r\n", 
            (huart->Instance == USART1) ? 1 : 
            (huart->Instance == USART3) ? 3 : 
            (huart->Instance == USART6) ? 6 : 
            (huart->Instance == UART4) ? 4 : 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    
    // 发送完成处理，不需要额外操作
}

// UART DMA相关函数已移至dma.c文件中实现
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
