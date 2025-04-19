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
#include "stdlib.h"
#include "iwdg.h"
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "CRC16_MODBUS.h"
#include "msg_list.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Ctrl_Msg_Func_Code
{
  GET_DATA = 0x01,
  MOTION_STATUS,
  RETURN_ZERO,
  ANGLE_AND_SPEED_CMD,
  DST_ANGLE_CMD,
  EMERGENCY_STOP,
  WIPERS_CTRL,
  LASER_CTRL,
  MOTOR_ERROR_CLEAR,
  RELATIVE_MOTION,
  OTA_BEGIN = 0x10,
  OTA_TRANSMIT,
  OTA_GET_STATE,
  OTA_FINISH,
  OTA_RESET
};

enum OTA_STATUS
{
  OTA_STATUS_NOT_STARTED = 0x00,
  OTA_STATUS_STARTED,
  OTA_STATUS_UPDATING,
  OTA_STATUS_SUCCESS,
  OTA_STATUS_FAIL,
  OTA_STATUS_NOT_STARTED_IN_BOOT,
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PITCH_PULSE_PER_DEG 1305.333333F
#define YAW_PULSE_PER_DEG 3533.402941F
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
uint8_t g_gimble_id = 1;
int16_t g_pitch_start_dst = 0; // 0.1 degree
int16_t g_pitch_end_dst = 0;   // 0.1 degree
int16_t g_pitch_speed_dst = 0; // deg/s
int16_t g_yaw_start_dst = 0;   // 0.1 degree
int16_t g_yaw_end_dst = 0;     // 0.1 degree
int16_t g_yaw_speed_dst = 0;   // deg/s
uint8_t g_motion_mode = 0;     // 0 dst point mode 1 start-end-speed mode 2 relative mode
uint8_t g_motion_request = 0;
uint8_t g_motion_step = 0;
int32_t g_pitch_start_dst_raw = 0; // encoder raw data
int32_t g_pitch_end_dst_raw = 0;   // encoder raw data
int16_t g_pitch_speed_dst_raw = 0; // encoder raw data
int32_t g_yaw_start_dst_raw = 0;   // encoder raw data
int32_t g_yaw_end_dst_raw = 0;     // encoder raw data
int16_t g_yaw_speed_dst_raw = 0;   // encoder raw data
int32_t g_pitch_offset = 0;
int32_t g_yaw_offset = 0;

extern volatile uint8_t g_rs485_c2_state;
extern volatile uint8_t g_rs485_c2_tx_buf[RS485_C2_TX_DATA_LENGTH];
extern volatile uint8_t g_rs485_c2_rx_buf[RS485_C2_RX_DATA_LENGTH];
volatile uint32_t g_tdlas_ppm = 0;
volatile uint8_t g_tdlas_laser_request = 0; // 0 no request 1 open request 2 close request

extern CAN_TxHeaderTypeDef g_can_tx_message_head;
extern volatile uint8_t g_can_tx_data[8];
extern CAN_RxHeaderTypeDef g_can_rx_message_head;
extern volatile uint8_t g_can_rx_data[8];
int16_t g_yaw = 0;           // 0.1 degree
int16_t g_pitch = 0;         // 0.1 degree
uint8_t g_motion_status = 0; // bit0 pitch motion status bit1 yaw motion status
int32_t g_yaw_raw = 0;       // encoder raw data
int32_t g_pitch_raw = 0;     // encoder raw data
int16_t g_pitch_speed_raw = 0;
int16_t g_yaw_speed_raw = 0;
uint16_t g_stop_speed_th = 10;
uint8_t g_motor_init_flag = 0;        // 0 not init 1 finished init
uint8_t g_zero_find_request = 0;      // 0 no request 1 request
uint8_t g_motor_zero_finded_flag = 0; // bit0 pitch state bit1 yaw state

volatile uint16_t g_pwm_min = 1000;
volatile uint16_t g_pwm_mid = 1500;
volatile uint16_t g_pwm_max = 2000;
volatile uint8_t g_pwm_flag = 0; // 0 stop 1 active once 2 open 255 fixing mode

uint8_t g_mcu_reset_request = 0;

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
void Ctrl_Msg_Decoding(Ctrl_Com_Msg *msg);
void Motor_Init(void);
void Motor_Zero_Find(void);
void Motor_Motion_Ctrl(void);
int32_t Motor_Angle_Raw_Limit(int32_t angle_raw, int32_t max, int32_t min);
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
    if (0 == g_motor_init_flag)
    {
      Motor_Init();
    }
    if (g_zero_find_request == 1)
    {
      Motor_Zero_Find();
    }
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
  Ctrl_Com_Msg ctrl_tx_msg_temp;
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

    if (0 == Ctrl_Tx_Msg_Get(&ctrl_tx_msg_temp))
    {
      RS485_Status_Set(RS485_CH1, RS485_WRITE);
      g_rs485_c1_tx_buf[0] = ctrl_tx_msg_temp.head_1;
      g_rs485_c1_tx_buf[1] = ctrl_tx_msg_temp.head_2;
      g_rs485_c1_tx_buf[2] = ctrl_tx_msg_temp.src_id;
      g_rs485_c1_tx_buf[3] = ctrl_tx_msg_temp.dst_id;
      g_rs485_c1_tx_buf[4] = ctrl_tx_msg_temp.func_code;
      g_rs485_c1_tx_buf[5] = ctrl_tx_msg_temp.data_len;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len; i++)
      {
        g_rs485_c1_tx_buf[i + 6] = ctrl_tx_msg_temp.data[i];
      }
      g_rs485_c1_tx_buf[ctrl_tx_msg_temp.data_len + 6] = ctrl_tx_msg_temp.check;
      g_rs485_c1_tx_buf[ctrl_tx_msg_temp.data_len + 7] = ctrl_tx_msg_temp.tail_1;
      g_rs485_c1_tx_buf[ctrl_tx_msg_temp.data_len + 8] = ctrl_tx_msg_temp.tail_2;
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&g_rs485_c1_tx_buf, ctrl_tx_msg_temp.data_len + 9);
      g_rs485_c1_state = 3;
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
  uint16_t crc16_check = 0;
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
          crc16_check = crc16_modbus(0xFFFF, (const unsigned char *)&g_rs485_c2_tx_buf, 6);
          tdlas_tx_msg_temp.crc_l = crc16_check & 0x00FF;
          tdlas_tx_msg_temp.crc_h = (crc16_check >> 8) & 0x00FF;
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
          crc16_check = crc16_modbus(0xFFFF, (const unsigned char *)&g_rs485_c2_tx_buf, 9);
          tdlas_tx_msg_temp.crc_l = crc16_check & 0x00FF;
          tdlas_tx_msg_temp.crc_h = (crc16_check >> 8) & 0x00FF;
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
  Motor_Tx_Ctrl_Msg motor_tx_msg_temp;
  uint32_t can_tx_box = 0;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);

    if (0 == Motor_Tx_Msg_Get(&motor_tx_msg_temp))
    {
      g_can_tx_message_head.StdId = motor_tx_msg_temp.head.StdId;
      g_can_tx_message_head.ExtId = motor_tx_msg_temp.head.ExtId;
      g_can_tx_message_head.IDE = motor_tx_msg_temp.head.IDE;
      g_can_tx_message_head.RTR = motor_tx_msg_temp.head.RTR;
      g_can_tx_message_head.DLC = motor_tx_msg_temp.head.DLC;
      g_can_tx_message_head.TransmitGlobalTime = motor_tx_msg_temp.head.TransmitGlobalTime;
      for (uint8_t i = 0; i < 8; i++)
      {
        g_can_tx_data[i] = motor_tx_msg_temp.data[i];
      }
      HAL_CAN_AddTxMessage(&hcan, &g_can_tx_message_head, (uint8_t *)g_can_tx_data, &can_tx_box);
    }
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
  uint16_t wait_cnt = 0;
  Rangefinder_Msg range_msg_temp;
  TDLAS_Tx_Msg tdlas_tx_msg_temp;
  TDLAS_Rx_Msg tdlas_rx_msg_temp;
  Ctrl_Com_Msg ctrl_rx_msg_temp;
  Motor_Rx_Ctrl_Msg motor_rx_msg_temp;
  uint16_t crc16_check = 0;
  uint8_t sum_check = 0;

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

    if (0 == Motor_Rx_Msg_Get(&motor_rx_msg_temp))
    {
      switch (motor_rx_msg_temp.head.StdId)
      {
      case 0x181: // pitch TPDO1
        if ((motor_rx_msg_temp.data[1] >> 2) && 0x01)
        {
          g_motion_status &= ~(0x01);
        }
        else
        {
          g_motion_status |= 0x01;
        }

        g_pitch_speed_raw = motor_rx_msg_temp.data[2];
        g_pitch_speed_raw |= (((uint16_t)motor_rx_msg_temp.data[3] << 8) & 0xFF00);

        // if (abs(g_pitch_speed_raw) < g_stop_speed_th)
        // {
        //   g_motion_status &= ~(0x01);
        // }
        // else
        // {
        //   g_motion_status |= 0x01;
        // }

        g_pitch_raw = motor_rx_msg_temp.data[4];
        g_pitch_raw |= (((uint32_t)motor_rx_msg_temp.data[5] << 8) & 0x0000FF00);
        g_pitch_raw |= (((uint32_t)motor_rx_msg_temp.data[6] << 16) & 0x00FF0000);
        g_pitch_raw |= (((uint32_t)motor_rx_msg_temp.data[7] << 24) & 0xFF000000);

        // here need trans raw pitch to degree pitch
        g_pitch = (int16_t)((g_pitch_raw - g_pitch_offset) / PITCH_PULSE_PER_DEG * 10.0F);
        break;
      case 0x182: // yaw TPDO1
        if ((motor_rx_msg_temp.data[1] >> 2) && 0x01)
        {
          g_motion_status &= ~(0x02);
        }
        else
        {
          g_motion_status |= 0x02;
        }

        g_yaw_speed_raw = motor_rx_msg_temp.data[2];
        g_yaw_speed_raw |= (((uint16_t)motor_rx_msg_temp.data[3] << 8) & 0xFF00);

        // if (abs(g_yaw_speed_raw) < g_stop_speed_th)
        // {
        //   g_motion_status &= ~(0x02);
        // }
        // else
        // {
        //   g_motion_status |= 0x02;
        // }

        g_yaw_raw = motor_rx_msg_temp.data[4];
        g_yaw_raw |= (((uint32_t)motor_rx_msg_temp.data[5] << 8) & 0x0000FF00);
        g_yaw_raw |= (((uint32_t)motor_rx_msg_temp.data[6] << 16) & 0x00FF0000);
        g_yaw_raw |= (((uint32_t)motor_rx_msg_temp.data[7] << 24) & 0xFF000000);

        // here need trans raw yaw to degree yaw
        g_yaw = (int16_t)((g_yaw_raw - g_yaw_offset) / YAW_PULSE_PER_DEG * 10.0F);
        break;
      case 0x581: // pitch SDO
        if (0x4B == motor_rx_msg_temp.data[0] && 0x09 == motor_rx_msg_temp.data[1] && 0x20 == motor_rx_msg_temp.data[2] && 0x00 == motor_rx_msg_temp.data[3])
        {
          if (motor_rx_msg_temp.data[4])
          {
            g_motor_zero_finded_flag &= ~(0x01);
          }
          else
          {
            g_motor_zero_finded_flag |= 0x01;
          }
        }
        break;
      case 0x582: // yaw SDO
        if (0x4B == motor_rx_msg_temp.data[0] && 0x09 == motor_rx_msg_temp.data[1] && 0x20 == motor_rx_msg_temp.data[2] && 0x00 == motor_rx_msg_temp.data[3])
        {
          if (motor_rx_msg_temp.data[4])
          {
            g_motor_zero_finded_flag &= ~(0x02);
          }
          else
          {
            g_motor_zero_finded_flag |= 0x02;
          }
        }
        break;

      default:
        break;
      }
    }

    if (0 == Rangerfinder_Msg_Get(&range_msg_temp)) // distance decode
    {
      if (0x03 == range_msg_temp.func_code && 0x04 == range_msg_temp.data_len)
      {
        crc16_check = crc16_modbus(0xFFFF, (const unsigned char *)&range_msg_temp, RS232_RX_DATA_LENGTH - 2);
        if ((crc16_check & 0x00FF) == range_msg_temp.crc_l && ((crc16_check >> 8) & 0x00FF) == range_msg_temp.crc_h)
        {
          g_distance = range_msg_temp.data[3];
          g_distance |= (range_msg_temp.data[2] << 8);
          g_distance |= (range_msg_temp.data[1] << 16);
          g_distance |= (range_msg_temp.data[0] << 24);
        }
      }
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
        crc16_check = crc16_modbus(0xFFFF, (const unsigned char *)&data_buf_temp, tdlas_rx_msg_temp.data_len + 3);
        if ((crc16_check & 0x00FF) == tdlas_rx_msg_temp.crc_l && ((crc16_check >> 8) & 0x00FF) == tdlas_rx_msg_temp.crc_h)
        {
          g_tdlas_ppm = tdlas_rx_msg_temp.data[3];
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[2] << 8);
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[1] << 16);
          g_tdlas_ppm |= (tdlas_rx_msg_temp.data[0] << 24);
        }
      }
    }

    if (0 == Ctrl_Rx_Msg_Get(&ctrl_rx_msg_temp))
    {
      uint8_t *ptr = &ctrl_rx_msg_temp.head_1;
      sum_check = 0;
      for (uint8_t i = 0; i < ctrl_rx_msg_temp.data_len + 6; i++)
      {
        sum_check += *(ptr + i);
      }
      if (ctrl_rx_msg_temp.check == sum_check && 0x0D == ctrl_rx_msg_temp.tail_1 && 0x0A == ctrl_rx_msg_temp.tail_2)
      {
        if (g_gimble_id == ctrl_rx_msg_temp.dst_id)
        {
          Ctrl_Msg_Decoding(&ctrl_rx_msg_temp);
        }
      }
    }

    if (g_mcu_reset_request)
    {
      wait_cnt++;
      if (wait_cnt >= 10)
      {
        wait_cnt = 0;
        MCU_Reset();
      }
    }
    else
    {
      wait_cnt = 0;
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

    if (0 != g_motion_request)
    {
      if (g_motor_init_flag > 1)
      {
        Motor_Motion_Ctrl();
      }
      else
      {
        g_motion_request = 0;
        g_motion_step = 0;
      }
    }
    else
    {
      g_motion_step = 0;
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
  uint16_t pulse_temp = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    switch (g_pwm_flag)
    {
    case 0:
      if (g_pwm_min - 1 != pulse_temp)
      {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        pulse_temp = g_pwm_min - 1;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
        osDelay(1000);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
      }
      break;
    case 1: // active once
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      pulse_temp = g_pwm_max;
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
      osDelay(500);
      pulse_temp = g_pwm_min;
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
      g_pwm_flag = 0;
      break;
    case 2: // open
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      pulse_temp = g_pwm_max;
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
      osDelay(500);
      pulse_temp = g_pwm_min;
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
      osDelay(1000);
      break;
    case 255: // fixing mode
      if (g_pwm_mid != pulse_temp)
      {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        pulse_temp = g_pwm_mid;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_temp);
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
void Ctrl_Msg_Decoding(Ctrl_Com_Msg *msg)
{
  Ctrl_Com_Msg ctrl_tx_msg_temp;
  uint8_t *ptr = &ctrl_tx_msg_temp.head_1;
  Motor_Tx_Ctrl_Msg motor_tx_msg_temp;
  uint32_t fw_size = 0;

  switch (msg->func_code)
  {
  case GET_DATA:
    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = GET_DATA;
    ctrl_tx_msg_temp.data_len = 0x0C;
    ctrl_tx_msg_temp.data[0] = (uint8_t)(g_distance & 0x000000FF);
    ctrl_tx_msg_temp.data[1] = (uint8_t)((g_distance >> 8) & 0x000000FF);
    ctrl_tx_msg_temp.data[2] = (uint8_t)((g_distance >> 16) & 0x000000FF);
    ctrl_tx_msg_temp.data[3] = (uint8_t)((g_distance >> 24) & 0x000000FF);
    ctrl_tx_msg_temp.data[4] = (uint8_t)(g_tdlas_ppm & 0x000000FF);
    ctrl_tx_msg_temp.data[5] = (uint8_t)((g_tdlas_ppm >> 8) & 0x000000FF);
    ctrl_tx_msg_temp.data[6] = (uint8_t)((g_tdlas_ppm >> 16) & 0x000000FF);
    ctrl_tx_msg_temp.data[7] = (uint8_t)((g_tdlas_ppm >> 24) & 0x000000FF);
    ctrl_tx_msg_temp.data[8] = (uint8_t)(g_yaw & 0x00FF);
    ctrl_tx_msg_temp.data[9] = (uint8_t)((g_yaw >> 8) & 0x00FF);
    ctrl_tx_msg_temp.data[10] = (uint8_t)(g_pitch & 0x00FF);
    ctrl_tx_msg_temp.data[11] = (uint8_t)((g_pitch >> 8) & 0x00FF);
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    break;
  case MOTION_STATUS:
    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = MOTION_STATUS;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = g_motion_status;
    if (0 == g_motion_status && 0 != g_motion_request) // avoid report stop status when consecutive movement
    {
      ctrl_tx_msg_temp.data[0] = 0x03;
    }
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    break;
  case RETURN_ZERO:
    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = RETURN_ZERO;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = 0x01;
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    // zero find cmd
    if (g_motor_init_flag > 1)
    {
      g_motor_zero_finded_flag = 0;
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x2B;
      motor_tx_msg_temp.data[1] = 0x09;
      motor_tx_msg_temp.data[2] = 0x20;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = 0x01;
      motor_tx_msg_temp.data[5] = 0x00;
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      g_zero_find_request = 1;
    }
    break;
  case ANGLE_AND_SPEED_CMD:
    if (0 == g_motion_request && g_motor_zero_finded_flag >= 3)
    {
      g_yaw_start_dst = msg->data[1];
      g_yaw_start_dst = (((g_yaw_start_dst << 8) & 0xFF00) | msg->data[0]);
      g_yaw_end_dst = msg->data[3];
      g_yaw_end_dst = (((g_yaw_end_dst << 8) & 0xFF00) | msg->data[2]);
      g_yaw_speed_dst = msg->data[5];
      g_yaw_speed_dst = (((g_yaw_speed_dst << 8) & 0xFF00) | msg->data[4]);
      g_pitch_start_dst = msg->data[7];
      g_pitch_start_dst = (((g_pitch_start_dst << 8) & 0xFF00) | msg->data[6]);
      g_pitch_end_dst = msg->data[9];
      g_pitch_end_dst = (((g_pitch_end_dst << 8) & 0xFF00) | msg->data[8]);
      g_pitch_speed_dst = msg->data[11];
      g_pitch_speed_dst = (((g_pitch_speed_dst << 8) & 0xFF00) | msg->data[10]);

      // here nedd trans dst angle to dst raw angle
      g_yaw_start_dst_raw = (int32_t)(g_yaw_start_dst / 10.0F * YAW_PULSE_PER_DEG) + g_yaw_offset;
      g_yaw_start_dst_raw = Motor_Angle_Raw_Limit(g_yaw_start_dst_raw, 165 * YAW_PULSE_PER_DEG, -165 * YAW_PULSE_PER_DEG);
      g_yaw_end_dst_raw = (int32_t)(g_yaw_end_dst / 10.0F * YAW_PULSE_PER_DEG) + g_yaw_offset;
      g_yaw_end_dst_raw = Motor_Angle_Raw_Limit(g_yaw_end_dst_raw, 165 * YAW_PULSE_PER_DEG, -165 * YAW_PULSE_PER_DEG);
      g_yaw_speed_dst_raw = (int16_t)(g_yaw_speed_dst * 60.0F / 360.0F * 72.0F / 28.0F);
      if (g_yaw_speed_dst_raw > 5000)
        g_yaw_speed_dst_raw = 5000;
      if (g_yaw_speed_dst_raw < 0)
        g_yaw_speed_dst_raw = 0;
      g_pitch_start_dst_raw = (int32_t)(g_pitch_start_dst / 10.0F * PITCH_PULSE_PER_DEG) + g_pitch_offset;
      g_pitch_start_dst_raw = Motor_Angle_Raw_Limit(g_pitch_start_dst_raw, 85 * PITCH_PULSE_PER_DEG, -85 * PITCH_PULSE_PER_DEG);
      g_pitch_end_dst_raw = (int32_t)(g_pitch_end_dst / 10.0F * PITCH_PULSE_PER_DEG) + g_pitch_offset;
      g_pitch_end_dst_raw = Motor_Angle_Raw_Limit(g_pitch_end_dst_raw, 85 * PITCH_PULSE_PER_DEG, -85 * PITCH_PULSE_PER_DEG);
      g_pitch_speed_dst_raw = (int16_t)(g_pitch_speed_dst * 60.0F / 360.0F * 60.0F / 34.0F);
      if (g_pitch_speed_dst_raw > 5000)
        g_pitch_speed_dst_raw = 5000;
      if (g_pitch_speed_dst_raw < 0)
        g_pitch_speed_dst_raw = 0;

      g_motion_mode = 1;
      g_motion_request = 1;

      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = ANGLE_AND_SPEED_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x01;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    else
    {
      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = ANGLE_AND_SPEED_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x00;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    break;
  case DST_ANGLE_CMD:
    if (0 == g_motion_request && g_motor_zero_finded_flag >= 3)
    {
      g_yaw_end_dst = msg->data[1];
      g_yaw_end_dst = (((g_yaw_end_dst << 8) & 0xFF00) | msg->data[0]);
      g_pitch_end_dst = msg->data[3];
      g_pitch_end_dst = (((g_pitch_end_dst << 8) & 0xFF00) | msg->data[2]);

      // here nedd trans dst angle to dst raw angle
      g_yaw_end_dst_raw = (int32_t)(g_yaw_end_dst / 10.0F * YAW_PULSE_PER_DEG) + g_yaw_offset;
      g_yaw_end_dst_raw = Motor_Angle_Raw_Limit(g_yaw_end_dst_raw, 165 * YAW_PULSE_PER_DEG, -165 * YAW_PULSE_PER_DEG);
      g_pitch_end_dst_raw = (int32_t)(g_pitch_end_dst / 10.0F * PITCH_PULSE_PER_DEG) + g_pitch_offset;
      g_pitch_end_dst_raw = Motor_Angle_Raw_Limit(g_pitch_end_dst_raw, 85 * PITCH_PULSE_PER_DEG, -85 * PITCH_PULSE_PER_DEG);

      g_motion_mode = 0;
      g_motion_request = 1;

      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = DST_ANGLE_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x01;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    else
    {
      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = DST_ANGLE_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x00;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    break;
  case EMERGENCY_STOP:
    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = EMERGENCY_STOP;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = 0x01;
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    // send emergency cmd
    if (g_motor_init_flag > 1)
    {
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x2B;
      motor_tx_msg_temp.data[1] = 0x03;
      motor_tx_msg_temp.data[2] = 0x20;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = 0x01;
      motor_tx_msg_temp.data[5] = 0x00;
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    }
    break;
  case WIPERS_CTRL:
    g_pwm_flag = msg->data[0];

    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = WIPERS_CTRL;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = msg->data[0];
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    break;
  case LASER_CTRL:
    if (msg->data[0])
    {
      g_tdlas_laser_request = 1;
    }
    else
    {
      g_tdlas_laser_request = 2;
    }

    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = LASER_CTRL;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = msg->data[0];
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    break;
  case MOTOR_ERROR_CLEAR:
    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = MOTOR_ERROR_CLEAR;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = 0x01;
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    // need send error clear cmd
    motor_tx_msg_temp.head.StdId = 0x201;
    motor_tx_msg_temp.head.ExtId = 0x201;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 6;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x86;
    motor_tx_msg_temp.data[1] = 0x00;
    motor_tx_msg_temp.data[2] = 0x00;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x00;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    motor_tx_msg_temp.head.StdId = 0x202;
    motor_tx_msg_temp.head.ExtId = 0x202;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    // enable motor
    motor_tx_msg_temp.head.StdId = 0x201;
    motor_tx_msg_temp.head.ExtId = 0x201;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 6;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x07;
    motor_tx_msg_temp.data[1] = 0x00;
    motor_tx_msg_temp.data[2] = 0x00;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x00;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    motor_tx_msg_temp.head.StdId = 0x202;
    motor_tx_msg_temp.head.ExtId = 0x202;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    break;
  case RELATIVE_MOTION:
    if (0 == g_motion_request)
    {
      g_yaw_end_dst = msg->data[1];
      g_yaw_end_dst = (((g_yaw_end_dst << 8) & 0xFF00) | msg->data[0]);
      g_pitch_end_dst = msg->data[3];
      g_pitch_end_dst = (((g_pitch_end_dst << 8) & 0xFF00) | msg->data[2]);

      // here nedd trans dst angle to dst raw angle
      g_yaw_end_dst_raw = (int32_t)(g_yaw_end_dst / 10.0F * YAW_PULSE_PER_DEG) + g_yaw_offset;
      g_yaw_end_dst_raw = Motor_Angle_Raw_Limit(g_yaw_end_dst_raw, 165 * YAW_PULSE_PER_DEG, -165 * YAW_PULSE_PER_DEG);
      g_pitch_end_dst_raw = (int32_t)(g_pitch_end_dst / 10.0F * PITCH_PULSE_PER_DEG) + g_pitch_offset;
      g_pitch_end_dst_raw = Motor_Angle_Raw_Limit(g_pitch_end_dst_raw, 85 * PITCH_PULSE_PER_DEG, -85 * PITCH_PULSE_PER_DEG);

      g_motion_mode = 2;
      g_motion_request = 1;

      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = DST_ANGLE_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x01;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    else
    {
      ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
      ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
      ctrl_tx_msg_temp.src_id = g_gimble_id;
      ctrl_tx_msg_temp.dst_id = msg->src_id;
      ctrl_tx_msg_temp.func_code = DST_ANGLE_CMD;
      ctrl_tx_msg_temp.data_len = 0x01;
      ctrl_tx_msg_temp.data[0] = 0x00;
      ctrl_tx_msg_temp.check = 0;
      for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
      {
        ctrl_tx_msg_temp.check += *(ptr + i);
      }
      ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
      ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
      Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);
    }
    break;
  case OTA_BEGIN:
    fw_size = *(uint32_t *)&msg->data[0];
    Save_OTA_Flag(fw_size);

    ctrl_tx_msg_temp.head_1 = CTRL_MSG_HEAD_1;
    ctrl_tx_msg_temp.head_2 = CTRL_MSG_HEAD_2;
    ctrl_tx_msg_temp.src_id = g_gimble_id;
    ctrl_tx_msg_temp.dst_id = msg->src_id;
    ctrl_tx_msg_temp.func_code = OTA_BEGIN;
    ctrl_tx_msg_temp.data_len = 0x01;
    ctrl_tx_msg_temp.data[0] = OTA_STATUS_NOT_STARTED;
    ctrl_tx_msg_temp.check = 0;
    for (uint8_t i = 0; i < ctrl_tx_msg_temp.data_len + 6; i++)
    {
      ctrl_tx_msg_temp.check += *(ptr + i);
    }
    ctrl_tx_msg_temp.tail_1 = CTRL_MSG_TAIL_1;
    ctrl_tx_msg_temp.tail_2 = CTRL_MSG_TAIL_2;
    Ctrl_Tx_Msg_Add(&ctrl_tx_msg_temp);

    g_mcu_reset_request = 1;
    break;

  default:
    break;
  }
}

void Motor_Init(void)
{
  static uint8_t step = 0;
  static uint8_t wait_cnt = 0;
  Motor_Tx_Ctrl_Msg motor_tx_msg_temp;

  switch (step)
  {
  case 0:
    motor_tx_msg_temp.head.StdId = 0x0000;
    motor_tx_msg_temp.head.ExtId = 0x0000;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 2;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x01;
    motor_tx_msg_temp.data[1] = 0x00;
    motor_tx_msg_temp.data[2] = 0x00;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x00;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    step++;
    break;
  case 1:
    motor_tx_msg_temp.head.StdId = 0x201;
    motor_tx_msg_temp.head.ExtId = 0x201;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 6;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x06;
    motor_tx_msg_temp.data[1] = 0x00;
    motor_tx_msg_temp.data[2] = 0x00;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x00;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    motor_tx_msg_temp.head.StdId = 0x202;
    motor_tx_msg_temp.head.ExtId = 0x202;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    step++;
    break;
  case 2:
    motor_tx_msg_temp.head.StdId = 0x201;
    motor_tx_msg_temp.head.ExtId = 0x201;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 6;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x07;
    motor_tx_msg_temp.data[1] = 0x00;
    motor_tx_msg_temp.data[2] = 0x00;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x00;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    motor_tx_msg_temp.head.StdId = 0x202;
    motor_tx_msg_temp.head.ExtId = 0x202;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    wait_cnt = 0;
    step++;
    break;
  case 3:
    wait_cnt++;
    if (wait_cnt >= 2)
    {
      wait_cnt = 0;
      // step++;
      step = 0;
      g_motor_init_flag = 1;
      g_motor_zero_finded_flag = 0;
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x2B;
      motor_tx_msg_temp.data[1] = 0x09;
      motor_tx_msg_temp.data[2] = 0x20;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = 0x01;
      motor_tx_msg_temp.data[5] = 0x00;
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      g_zero_find_request = 1;
    }
    break;
  case 4:
    g_motor_zero_finded_flag = 0;
    motor_tx_msg_temp.head.StdId = 0x601;
    motor_tx_msg_temp.head.ExtId = 0x601;
    motor_tx_msg_temp.head.IDE = CAN_ID_STD;
    motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
    motor_tx_msg_temp.head.DLC = 6;
    motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
    motor_tx_msg_temp.data[0] = 0x2B;
    motor_tx_msg_temp.data[1] = 0x09;
    motor_tx_msg_temp.data[2] = 0x20;
    motor_tx_msg_temp.data[3] = 0x00;
    motor_tx_msg_temp.data[4] = 0x01;
    motor_tx_msg_temp.data[5] = 0x00;
    motor_tx_msg_temp.data[6] = 0x00;
    motor_tx_msg_temp.data[7] = 0x00;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    motor_tx_msg_temp.head.StdId = 0x602;
    motor_tx_msg_temp.head.ExtId = 0x602;
    Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    step++;
    break;
  case 5:
    wait_cnt++;
    if (wait_cnt >= 2)
    {
      wait_cnt = 0;
      step++;
    }
    break;
  case 6:
    if (g_motor_zero_finded_flag < 3)
    {
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 8;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x40;
      motor_tx_msg_temp.data[1] = 0x09;
      motor_tx_msg_temp.data[2] = 0x20;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = 0x00;
      motor_tx_msg_temp.data[5] = 0x00;
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    }
    else
    {
      step = 0;
      g_motor_init_flag = 2;
    }
    break;

  default:
    break;
  }
}

void Motor_Zero_Find(void)
{
  static uint8_t step = 0;
  static uint8_t wait_cnt = 0;
  Motor_Tx_Ctrl_Msg motor_tx_msg_temp;

  switch (step)
  {
  case 0:
    wait_cnt++;
    if (wait_cnt >= 2)
    {
      wait_cnt = 0;
      step++;
    }
    break;
  case 1:
    if (g_motor_zero_finded_flag < 3)
    {
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 8;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x40;
      motor_tx_msg_temp.data[1] = 0x09;
      motor_tx_msg_temp.data[2] = 0x20;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = 0x00;
      motor_tx_msg_temp.data[5] = 0x00;
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
    }
    else
    {
      step = 0;
      g_zero_find_request = 0;
      g_motor_init_flag = 2;
    }
    break;

  default:
    step = 0;
    g_zero_find_request = 0;
    break;
  }
}

void Motor_Motion_Ctrl(void)
{
  static uint32_t wait_cnt = 0;
  Motor_Tx_Ctrl_Msg motor_tx_msg_temp;

  switch (g_motion_mode)
  {
  case 0: // dst angle ctrl
    if (g_motor_zero_finded_flag < 3)
    {
      g_motion_step = 0;
      g_motion_request = 0;
    }
    else
    {
      switch (g_motion_step)
      {
      case 0:
        // enable motor first
        motor_tx_msg_temp.head.StdId = 0x201;
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x0F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202;
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        g_motion_step++;
        break;
      case 1:
        g_pitch_speed_dst_raw = 3000;
        g_yaw_speed_dst_raw = 3000;
        // need to set motion speed
        motor_tx_msg_temp.head.StdId = 0x601;
        motor_tx_msg_temp.head.ExtId = 0x601;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x2B;
        motor_tx_msg_temp.data[1] = 0x7F;
        motor_tx_msg_temp.data[2] = 0x60;
        motor_tx_msg_temp.data[3] = 0x00;
        motor_tx_msg_temp.data[4] = (uint8_t)(g_pitch_speed_dst_raw & 0x00FF);
        motor_tx_msg_temp.data[5] = (uint8_t)((g_pitch_speed_dst_raw >> 8) & 0x00FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x602;
        motor_tx_msg_temp.head.ExtId = 0x602;
        motor_tx_msg_temp.data[4] = (uint8_t)(g_yaw_speed_dst_raw & 0x00FF);
        motor_tx_msg_temp.data[5] = (uint8_t)((g_yaw_speed_dst_raw >> 8) & 0x00FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        g_motion_step++;
        break;
      case 2:
        motor_tx_msg_temp.head.StdId = 0x201; // pitch
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x5F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202; // yaw
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        wait_cnt = 0;
        g_motion_step++;
        break;
      case 3:
        wait_cnt++;
        if (wait_cnt >= 10)
        {
          wait_cnt = 0;
          g_motion_step++;
        }
        break;
      case 4:
        if (0 == g_motion_status)
        {
          g_motion_step = 0;
          g_motion_request = 0;
        }
        break;

      default:
        break;
      }
    }
    break;
  case 1: // start-end-speed ctrl
    if (g_motor_zero_finded_flag < 3)
    {
      g_motion_step = 0;
      g_motion_request = 0;
    }
    else
    {
      switch (g_motion_step)
      {
      case 0:
        // enable motor first
        motor_tx_msg_temp.head.StdId = 0x201;
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x0F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_start_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_start_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_start_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_start_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202;
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_start_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_start_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_start_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_start_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        g_motion_step++;
        break;
      case 1:
        // need to set motion speed
        motor_tx_msg_temp.head.StdId = 0x601;
        motor_tx_msg_temp.head.ExtId = 0x601;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x2B;
        motor_tx_msg_temp.data[1] = 0x7F;
        motor_tx_msg_temp.data[2] = 0x60;
        motor_tx_msg_temp.data[3] = 0x00;
        motor_tx_msg_temp.data[4] = (uint8_t)(g_pitch_speed_dst_raw & 0x00FF);
        motor_tx_msg_temp.data[5] = (uint8_t)((g_pitch_speed_dst_raw >> 8) & 0x00FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x602;
        motor_tx_msg_temp.head.ExtId = 0x602;
        motor_tx_msg_temp.data[4] = (uint8_t)(g_yaw_speed_dst_raw & 0x00FF);
        motor_tx_msg_temp.data[5] = (uint8_t)((g_yaw_speed_dst_raw >> 8) & 0x00FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        g_motion_step++;
        break;
      case 2:
        motor_tx_msg_temp.head.StdId = 0x201; // pitch
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x5F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_start_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_start_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_start_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_start_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202; // yaw
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_start_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_start_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_start_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_start_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        wait_cnt = 0;
        g_motion_step++;
        break;
      case 3:
        wait_cnt++;
        if (wait_cnt >= 100)
        {
          wait_cnt = 0;
          g_motion_step++;
        }
        break;
      case 4:
        if (0 == g_motion_status)
        {
          g_motion_step++;
        }
        break;
      case 5:
        // enable motor first
        motor_tx_msg_temp.head.StdId = 0x201;
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x0F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202;
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        g_motion_step++;
        break;
      case 6:
        motor_tx_msg_temp.head.StdId = 0x201; // pitch
        motor_tx_msg_temp.head.ExtId = 0x201;
        motor_tx_msg_temp.head.IDE = CAN_ID_STD;
        motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
        motor_tx_msg_temp.head.DLC = 6;
        motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
        motor_tx_msg_temp.data[0] = 0x5F;
        motor_tx_msg_temp.data[1] = 0x00;
        motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
        motor_tx_msg_temp.data[6] = 0x00;
        motor_tx_msg_temp.data[7] = 0x00;
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        motor_tx_msg_temp.head.StdId = 0x202; // yaw
        motor_tx_msg_temp.head.ExtId = 0x202;
        motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
        motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
        motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
        motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
        Motor_Tx_Msg_Add(&motor_tx_msg_temp);
        wait_cnt = 0;
        g_motion_step++;
        break;
      case 7:
        wait_cnt++;
        if (wait_cnt >= 100)
        {
          wait_cnt = 0;
          g_motion_step++;
        }
        break;
      case 8:
        if (0 == g_motion_status)
        {
          g_motion_step = 0;
          g_motion_request = 0;
        }
        break;

      default:
        break;
      }
    }
    break;
  case 2: // relative ctrl
    switch (g_motion_step)
    {
    case 0:
      // enable motor first
      motor_tx_msg_temp.head.StdId = 0x201;
      motor_tx_msg_temp.head.ExtId = 0x201;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x0F;
      motor_tx_msg_temp.data[1] = 0x00;
      motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
      motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
      motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
      motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x202;
      motor_tx_msg_temp.head.ExtId = 0x202;
      motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
      motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
      motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
      motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      g_motion_step++;
      break;
    case 1:
      g_pitch_speed_dst_raw = 3000;
      g_yaw_speed_dst_raw = 3000;
      // need to set motion speed
      motor_tx_msg_temp.head.StdId = 0x601;
      motor_tx_msg_temp.head.ExtId = 0x601;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x2B;
      motor_tx_msg_temp.data[1] = 0x7F;
      motor_tx_msg_temp.data[2] = 0x60;
      motor_tx_msg_temp.data[3] = 0x00;
      motor_tx_msg_temp.data[4] = (uint8_t)(g_pitch_speed_dst_raw & 0x00FF);
      motor_tx_msg_temp.data[5] = (uint8_t)((g_pitch_speed_dst_raw >> 8) & 0x00FF);
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x602;
      motor_tx_msg_temp.head.ExtId = 0x602;
      motor_tx_msg_temp.data[4] = (uint8_t)(g_yaw_speed_dst_raw & 0x00FF);
      motor_tx_msg_temp.data[5] = (uint8_t)((g_yaw_speed_dst_raw >> 8) & 0x00FF);
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      g_motion_step++;
      break;
    case 2:
      motor_tx_msg_temp.head.StdId = 0x201; // pitch
      motor_tx_msg_temp.head.ExtId = 0x201;
      motor_tx_msg_temp.head.IDE = CAN_ID_STD;
      motor_tx_msg_temp.head.RTR = CAN_RTR_DATA;
      motor_tx_msg_temp.head.DLC = 6;
      motor_tx_msg_temp.head.TransmitGlobalTime = DISABLE;
      motor_tx_msg_temp.data[0] = 0x1F;
      motor_tx_msg_temp.data[1] = 0x00;
      motor_tx_msg_temp.data[2] = (g_pitch_end_dst_raw & 0x000000FF);
      motor_tx_msg_temp.data[3] = ((g_pitch_end_dst_raw >> 8) & 0x000000FF);
      motor_tx_msg_temp.data[4] = ((g_pitch_end_dst_raw >> 16) & 0x000000FF);
      motor_tx_msg_temp.data[5] = ((g_pitch_end_dst_raw >> 24) & 0x000000FF);
      motor_tx_msg_temp.data[6] = 0x00;
      motor_tx_msg_temp.data[7] = 0x00;
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      motor_tx_msg_temp.head.StdId = 0x202; // yaw
      motor_tx_msg_temp.head.ExtId = 0x202;
      motor_tx_msg_temp.data[2] = (g_yaw_end_dst_raw & 0x000000FF);
      motor_tx_msg_temp.data[3] = ((g_yaw_end_dst_raw >> 8) & 0x000000FF);
      motor_tx_msg_temp.data[4] = ((g_yaw_end_dst_raw >> 16) & 0x000000FF);
      motor_tx_msg_temp.data[5] = ((g_yaw_end_dst_raw >> 24) & 0x000000FF);
      Motor_Tx_Msg_Add(&motor_tx_msg_temp);
      wait_cnt = 0;
      g_motion_step++;
      break;
    case 3:
      wait_cnt++;
      if (wait_cnt >= 10)
      {
        wait_cnt = 0;
        g_motion_step++;
      }
      break;
    case 4:
      if (0 == g_motion_status)
      {
        g_motion_step = 0;
        g_motion_request = 0;
      }
      break;

    default:
      break;
    }
    break;

  default:
    g_motion_step = 0;
    g_motion_request = 0;
    break;
  }
}

int32_t Motor_Angle_Raw_Limit(int32_t angle_raw, int32_t max, int32_t min)
{
  int32_t angle_temp = angle_raw;
  if (angle_temp > max)
  {
    angle_temp = max;
  }
  if (angle_temp < min)
  {
    angle_temp = min;
  }
  return angle_temp;
}

/* USER CODE END Application */
