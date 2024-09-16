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
#include "tim.h"
#include "CRC16_MODBUS.h"
#include "msg_list.h"
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
extern volatile uint8_t g_rs232_rx_state;
extern volatile uint8_t g_rs232_rx_buf[RS232_RX_DATA_LENGTH];
volatile uint32_t g_distance = 0; // 0.1mm

extern volatile uint8_t g_rs485_c1_state;
extern volatile uint8_t g_rs485_c1_tx_buf[RS485_C1_TX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c1_rx_buf[RS485_C1_RX_DATA_LENGTH];

extern volatile uint8_t g_rs485_c2_state;
extern volatile uint8_t g_rs485_c2_tx_buf[RS485_C2_TX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c2_rx_buf[RS485_C2_RX_DATA_LENGTH];
volatile uint32_t g_tdlas_ppm = 0;
volatile uint8_t g_tdlas_laser_request = 0; // 0 no request 1 open request 2 close request

extern CAN_TxHeaderTypeDef g_can_tx_message_head;
extern volatile uint8_t g_can_tx_data[8];
extern CAN_RxHeaderTypeDef g_can_rx_message_head;
extern volatile uint8_t g_can_rx_data[8];

volatile uint16_t g_pwm_min = 1000;
volatile uint16_t g_pwm_mid = 1500;
volatile uint16_t g_pwm_max = 2000;
volatile uint8_t g_pwm_flag = 0; // 0 stop 1 active once 2 open 255 fixing mode

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
/* Definitions for myTask_PWM */
osThreadId_t myTask_PWMHandle;
const osThreadAttr_t myTask_PWM_attributes = {
    .name = "myTask_PWM",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
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
void StartTask_PWM(void *argument);

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

  /* creation of myTask_PWM */
  myTask_PWMHandle = osThreadNew(StartTask_PWM, NULL, &myTask_PWM_attributes);

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
    osDelayUntil(xLastWakeTime + 20);
    if (0 == g_rs232_rx_state)
    {
      if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)g_rs232_rx_buf, RS232_RX_DATA_LENGTH) == HAL_OK)
      {
        g_rs232_rx_state = 1;
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

    if (0 == g_rs485_c1_state)
    {
      if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)g_rs485_c1_rx_buf, RS485_C1_RX_DATA_LENGTH) == HAL_OK)
      {
        g_rs485_c1_state = 1;
      }
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
  TDLAS_Tx_Msg tdlas_tx_msg_temp;
  uint16_t check = 0;
  uint16_t timeout_cnt = 0;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);

    if (0 != g_rs485_c2_state)
    {
      timeout_cnt++;
      if (timeout_cnt >= 8) // 40ms timeout
      {
        HAL_UART_AbortReceive(&huart2);
        RS485_Status_Set(RS485_CH2, RS485_WRITE);
        g_rs485_c2_state = 0;
        timeout_cnt = 0;
      }
    }
    else
    {
      timeout_cnt = 0;
    }

    if (0 == g_rs485_c2_state)
    {
      if (0 == TDLAS_Tx_Msg_Get(&tdlas_tx_msg_temp))
      {
        if (0x03 == tdlas_tx_msg_temp.func_code)
        {
          g_rs485_c2_tx_buf[0] = tdlas_tx_msg_temp.addr;
          g_rs485_c2_tx_buf[1] = tdlas_tx_msg_temp.func_code;
          g_rs485_c2_tx_buf[2] = (tdlas_tx_msg_temp.reg_addr >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[3] = tdlas_tx_msg_temp.reg_addr & 0x00FF;
          g_rs485_c2_tx_buf[4] = (tdlas_tx_msg_temp.reg_cnt >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[5] = tdlas_tx_msg_temp.reg_cnt & 0x00FF;
          check = crc16_modbus(0xFFFF, (const unsigned char *)&g_rs485_c2_tx_buf, 6);
          tdlas_tx_msg_temp.crc_l = check & 0x00FF;
          tdlas_tx_msg_temp.crc_h = (check >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[6] = tdlas_tx_msg_temp.crc_l;
          g_rs485_c2_tx_buf[7] = tdlas_tx_msg_temp.crc_h;
          HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&g_rs485_c2_tx_buf, 8);
          g_rs485_c2_state = 1;
        }
        else if (0x10 == tdlas_tx_msg_temp.func_code)
        {
          g_rs485_c2_tx_buf[0] = tdlas_tx_msg_temp.addr;
          g_rs485_c2_tx_buf[1] = tdlas_tx_msg_temp.func_code;
          g_rs485_c2_tx_buf[2] = (tdlas_tx_msg_temp.reg_addr >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[3] = tdlas_tx_msg_temp.reg_addr & 0x00FF;
          g_rs485_c2_tx_buf[4] = (tdlas_tx_msg_temp.reg_cnt >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[5] = tdlas_tx_msg_temp.reg_cnt & 0x00FF;
          g_rs485_c2_tx_buf[6] = tdlas_tx_msg_temp.data_len;
          g_rs485_c2_tx_buf[7] = tdlas_tx_msg_temp.data[0];
          g_rs485_c2_tx_buf[8] = tdlas_tx_msg_temp.data[1];
          check = crc16_modbus(0xFFFF, (const unsigned char *)&g_rs485_c2_tx_buf, 9);
          tdlas_tx_msg_temp.crc_l = check & 0x00FF;
          tdlas_tx_msg_temp.crc_h = (check >> 8) & 0x00FF;
          g_rs485_c2_tx_buf[9] = tdlas_tx_msg_temp.crc_l;
          g_rs485_c2_tx_buf[10] = tdlas_tx_msg_temp.crc_h;
          HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&g_rs485_c2_tx_buf, 11);
          g_rs485_c2_state = 1;
        }
      }
    }
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
  TickType_t xLastWakeTime;
  uint32_t period_cnt = 0;
  Rangefinder_Msg range_msg_temp;
  TDLAS_Tx_Msg tdlas_tx_msg_temp;
  TDLAS_Rx_Msg tdlas_rx_msg_temp;
  uint16_t check = 0;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);
    period_cnt++;
    if (period_cnt >= 4294967250)
    {
      period_cnt = 0;
    }

    if (0 == Rangerfinder_Msg_Get(&range_msg_temp)) // distance decode
    {
      if (0x03 == range_msg_temp.func_code && 0x04 == range_msg_temp.data_len)
      {
        check = crc16_modbus(0xFFFF, (const unsigned char *)&range_msg_temp, RS232_RX_DATA_LENGTH - 2);
        if ((check & 0x00FF) == range_msg_temp.crc_l && ((check >> 8) & 0x00FF) == range_msg_temp.crc_h)
        {
          g_distance = range_msg_temp.data[3];
          g_distance |= (range_msg_temp.data[2] << 8);
          g_distance |= (range_msg_temp.data[1] << 16);
          g_distance |= (range_msg_temp.data[0] << 24);
        }
      }
    }

    if (period_cnt % 10 == 0) // 5ms * 10 = 50ms send tdlas measure cmd
    {
      tdlas_tx_msg_temp.addr = 0x01;
      tdlas_tx_msg_temp.func_code = 0x03;
      tdlas_tx_msg_temp.reg_addr = 0x0001;
      tdlas_tx_msg_temp.reg_cnt = 0x0002;
      tdlas_tx_msg_temp.crc_l = 0x95;
      tdlas_tx_msg_temp.crc_h = 0xCB;
      TDLAS_Tx_Msg_Add(&tdlas_tx_msg_temp);
    }

    if (1 == g_tdlas_laser_request) // send open tdlas laser cmd
    {
      tdlas_tx_msg_temp.addr = 0x01;
      tdlas_tx_msg_temp.func_code = 0x10;
      tdlas_tx_msg_temp.reg_addr = 0x0006;
      tdlas_tx_msg_temp.reg_cnt = 0x0001;
      tdlas_tx_msg_temp.data_len = 0x02;
      tdlas_tx_msg_temp.data[0] = 0x00;
      tdlas_tx_msg_temp.data[1] = 0x01;
      tdlas_tx_msg_temp.crc_l = 0x67;
      tdlas_tx_msg_temp.crc_h = 0xF6;
      TDLAS_Tx_Msg_Add(&tdlas_tx_msg_temp);
      g_tdlas_laser_request = 0;
    }
    else if (2 == g_tdlas_laser_request) // send close tdlas laser cmd
    {
      tdlas_tx_msg_temp.addr = 0x01;
      tdlas_tx_msg_temp.func_code = 0x10;
      tdlas_tx_msg_temp.reg_addr = 0x0006;
      tdlas_tx_msg_temp.reg_cnt = 0x0001;
      tdlas_tx_msg_temp.data_len = 0x02;
      tdlas_tx_msg_temp.data[0] = 0x00;
      tdlas_tx_msg_temp.data[1] = 0x00;
      tdlas_tx_msg_temp.crc_l = 0xA6;
      tdlas_tx_msg_temp.crc_h = 0x36;
      TDLAS_Tx_Msg_Add(&tdlas_tx_msg_temp);
      g_tdlas_laser_request = 0;
    }

    if (0 == TDLAS_Rx_Msg_Get(&tdlas_rx_msg_temp)) // tdlas decode
    {
      uint8_t data_buf_temp[16] = {0};
      if (0x03 == tdlas_rx_msg_temp.func_code && 0x04 == tdlas_rx_msg_temp.data_len)
      {
        data_buf_temp[0] = tdlas_rx_msg_temp.addr;
        data_buf_temp[1] = tdlas_rx_msg_temp.func_code;
        data_buf_temp[2] = tdlas_rx_msg_temp.data_len;
        for (uint8_t i = 0; i < tdlas_rx_msg_temp.data_len; i++)
        {
          data_buf_temp[i + 3] = tdlas_rx_msg_temp.data[i];
        }
        check = crc16_modbus(0xFFFF, (const unsigned char *)&data_buf_temp, tdlas_rx_msg_temp.data_len + 3);
        if ((check & 0x00FF) == tdlas_rx_msg_temp.crc_l && ((check >> 8) & 0x00FF) == tdlas_rx_msg_temp.crc_h)
        {
          g_tdlas_ppm = tdlas_rx_msg_temp.data[3];
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[2] << 8);
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[1] << 16);
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[0] << 24);
        }
      }
    }
  }
  /* USER CODE END StartTask_WorkFlow */
}

/* USER CODE BEGIN Header_StartTask_PWM */
/**
 * @brief Function implementing the myTask_PWM thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_PWM */
void StartTask_PWM(void *argument)
{
  /* USER CODE BEGIN StartTask_PWM */
  TIM_OC_InitTypeDef config;
  config.OCMode = TIM_OCFAST_ENABLE;
  config.Pulse = g_pwm_mid;
  config.OCFastMode = TIM_OCFAST_ENABLE;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  config.OCIdleState = TIM_OCIDLESTATE_RESET;
  config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    switch (g_pwm_flag)
    {
    case 0:
      if (g_pwm_min != config.Pulse)
      {
        config.Pulse = g_pwm_min;
        HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      }
      break;
    case 1: // active once
      config.Pulse = g_pwm_max;
      HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      osDelay(500);
      config.Pulse = g_pwm_min;
      HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      g_pwm_flag = 0;
      break;
    case 2: // open
      config.Pulse = g_pwm_max;
      HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      osDelay(500);
      config.Pulse = g_pwm_min;
      HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      osDelay(1000);
      break;
    case 255: // fixing mode
      if (g_pwm_mid != config.Pulse)
      {
        config.Pulse = g_pwm_mid;
        HAL_TIM_PWM_ConfigChannel(&htim4, &config, TIM_CHANNEL_1);
      }
      break;

    default:
      break;
    }
  }
  /* USER CODE END StartTask_PWM */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
