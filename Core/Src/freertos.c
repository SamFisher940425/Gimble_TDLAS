/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "usart.h"
#include "can.h"
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

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile uint8_t g_rs232_state;
extern volatile uint8_t g_rs232_rx_buf[RS232_RX_DATA_LENGTH];
extern volatile uint32_t g_distance_uint32; // 0.1mm
extern volatile float g_distance_f32;       // mm
extern volatile uint8_t g_rs485_c1_state;
extern volatile uint8_t g_rs485_c1_tx_buf[RS485_C1_TX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c1_rx_buf[RS485_C1_RX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c2_state;
extern volatile uint8_t g_rs485_c2_tx_cnt;
extern volatile uint8_t g_rs485_c2_rx_cnt;
extern volatile uint8_t g_rs485_c2_tx_buf[RS485_C2_TX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c2_rx_buf[RS485_C2_RX_DATA_LENGTH];
extern volatile uint16_t g_tdlas_ppm;
extern CAN_TxHeaderTypeDef g_can_tx_message_head;
extern volatile uint8_t g_can_tx_data[8];
extern CAN_RxHeaderTypeDef g_can_rx_message_head;
extern volatile uint8_t g_can_rx_data[8];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTask_WDog */
osThreadId_t myTask_WDogHandle;
const osThreadAttr_t myTask_WDog_attributes = {
    .name = "myTask_WDog",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for myTask_232Rx */
osThreadId_t myTask_232RxHandle;
const osThreadAttr_t myTask_232Rx_attributes = {
    .name = "myTask_232Rx",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for myTask_485C1TR */
osThreadId_t myTask_485C1TRHandle;
const osThreadAttr_t myTask_485C1TR_attributes = {
    .name = "myTask_485C1TR",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityHigh3,
};
/* Definitions for myTask_485C2TR */
osThreadId_t myTask_485C2TRHandle;
const osThreadAttr_t myTask_485C2TR_attributes = {
    .name = "myTask_485C2TR",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};
/* Definitions for myTask_CAN */
osThreadId_t myTask_CANHandle;
const osThreadAttr_t myTask_CAN_attributes = {
    .name = "myTask_CAN",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for myTask_WorkFlow */
osThreadId_t myTask_WorkFlowHandle;
const osThreadAttr_t myTask_WorkFlow_attributes = {
    .name = "myTask_WorkFlow",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask_WDog(void *argument);
void StartTask_232Rx(void *argument);
void StartTask_485C1TR(void *argument);
void StartTask_485C2TR(void *argument);
void StartTask_CAN(void *argument);
void StartTask_WorkFlow(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask_WDog */
  myTask_WDogHandle = osThreadNew(StartTask_WDog, NULL, &myTask_WDog_attributes);

  /* creation of myTask_232Rx */
  myTask_232RxHandle = osThreadNew(StartTask_232Rx, NULL, &myTask_232Rx_attributes);

  /* creation of myTask_485C1TR */
  myTask_485C1TRHandle = osThreadNew(StartTask_485C1TR, NULL, &myTask_485C1TR_attributes);

  /* creation of myTask_485C2TR */
  myTask_485C2TRHandle = osThreadNew(StartTask_485C2TR, NULL, &myTask_485C2TR_attributes);

  /* creation of myTask_CAN */
  myTask_CANHandle = osThreadNew(StartTask_CAN, NULL, &myTask_CAN_attributes);

  /* creation of myTask_WorkFlow */
  myTask_WorkFlowHandle = osThreadNew(StartTask_WorkFlow, NULL, &myTask_WorkFlow_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(500);
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_WDog */
/**
 * @brief Function implementing the myTask_WDog thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_WDog */
void StartTask_WDog(void *argument)
{
  /* USER CODE BEGIN StartTask_WDog */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 300);
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END StartTask_WDog */
}

/* USER CODE BEGIN Header_StartTask_232Rx */
/**
 * @brief Function implementing the myTask_232Rx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_232Rx */
void StartTask_232Rx(void *argument)
{
  /* USER CODE BEGIN StartTask_232Rx */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);
    if (0 == g_rs232_state)
    {
      if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)g_rs232_rx_buf, RS232_RX_DATA_LENGTH) == HAL_OK)
      {
        g_rs232_state = 1;
      }
    }
  }
  /* USER CODE END StartTask_232Rx */
}

/* USER CODE BEGIN Header_StartTask_485C1TR */
/**
 * @brief Function implementing the myTask_485C1TR thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_485C1TR */
void StartTask_485C1TR(void *argument)
{
  /* USER CODE BEGIN StartTask_485C1TR */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)g_rs485_c1_rx_buf, RS485_C1_RX_DATA_LENGTH) == HAL_OK)
    {
      g_rs485_c1_state = 1;
    }
  }
  /* USER CODE END StartTask_485C1TR */
}

/* USER CODE BEGIN Header_StartTask_485C2TR */
/**
 * @brief Function implementing the myTask_485C2TR thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_485C2TR */
void StartTask_485C2TR(void *argument)
{
  /* USER CODE BEGIN StartTask_485C2TR */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 1000);
    RS485_Status_Set(RS485_CH2, RS485_WRITE);
    osDelay(1);
    g_rs485_c2_tx_cnt = 8;
    g_rs485_c2_rx_cnt = 7;
    g_rs485_c2_tx_buf[0] = 0x01;
    g_rs485_c2_tx_buf[1] = 0x03;
    g_rs485_c2_tx_buf[2] = 0x00;
    g_rs485_c2_tx_buf[3] = 0x01;
    g_rs485_c2_tx_buf[4] = 0x00;
    g_rs485_c2_tx_buf[5] = 0x01;
    g_rs485_c2_tx_buf[6] = 0xD5;
    g_rs485_c2_tx_buf[7] = 0xCA;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&g_rs485_c2_tx_buf, g_rs485_c2_tx_cnt);
    g_rs485_c2_state = 1;
  }
  /* USER CODE END StartTask_485C2TR */
}

/* USER CODE BEGIN Header_StartTask_CAN */
/**
 * @brief Function implementing the myTask_CAN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_CAN */
void StartTask_CAN(void *argument)
{
  /* USER CODE BEGIN StartTask_CAN */
  TickType_t xLastWakeTime;
  uint32_t can_tx_box = 0;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 1000);

    
    g_can_tx_message_head.IDE = CAN_ID_STD;
    g_can_tx_message_head.StdId = 101;
    g_can_tx_message_head.RTR = CAN_RTR_DATA;
    g_can_tx_message_head.TransmitGlobalTime = DISABLE;
    g_can_tx_message_head.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan, &g_can_tx_message_head, (uint8_t *)g_can_tx_data, &can_tx_box);
  }
  /* USER CODE END StartTask_CAN */
}

/* USER CODE BEGIN Header_StartTask_WorkFlow */
/**
 * @brief Function implementing the myTask_WorkFlow thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_WorkFlow */
void StartTask_WorkFlow(void *argument)
{
  /* USER CODE BEGIN StartTask_WorkFlow */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);

    if (2 == g_rs232_state)
    {
      g_distance_uint32 = g_rs232_rx_buf[6];
      g_distance_uint32 |= (g_rs232_rx_buf[5] << 8);
      g_distance_uint32 |= (g_rs232_rx_buf[4] << 16);
      g_distance_uint32 |= (g_rs232_rx_buf[3] << 24);
      g_distance_f32 = g_distance_uint32 / 10.0F;
      g_rs232_state = 0;
    }

    if (3 == g_rs485_c2_state)
    {
      g_tdlas_ppm = g_rs485_c2_rx_buf[4];
      g_tdlas_ppm |= (g_rs485_c2_rx_buf[3] << 8);
      g_rs485_c2_state = 0;
    }
  }
  /* USER CODE END StartTask_WorkFlow */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
