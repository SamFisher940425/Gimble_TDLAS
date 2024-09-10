/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CRC16_MODBUS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RS485_READ = 0,
  RS485_WRITE = 1
} RS485_Status;

typedef enum
{
  RS485_CH1 = 0,
  RS485_CH2 = 1
} RS485_Channel;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PELCOD_CMD_LENGTH 7
#define PELCOD_CMD_TYPE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_rs232_status = 0;
volatile uint8_t g_rs232_rx_buf[RS232_RX_DATA_LENGTH] = {0};
volatile uint32_t g_distance_uint32 = 0; // 0.1mm
volatile float g_distance_f32 = 0.0F; // mm
volatile uint8_t g_rs485_tx_buf[PELCOD_CMD_TYPE][PELCOD_CMD_LENGTH] = {0}; // line_0: move, line_1: zoom, line_2: focal, line_3: aperture, line_4: light, line_5: wiper
volatile uint8_t g_pelcod_status[PELCOD_CMD_TYPE] = {0};
volatile uint8_t g_pelcod_new_pack_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  __HAL_DBGMCU_FREEZE_IWDG();
	__HAL_DBGMCU_FREEZE_WWDG();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void RS232_RxEventCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos)
{
  HAL_UART_RxEventTypeTypeDef event_type = 3; // set a invalid value temp
  event_type = HAL_UARTEx_GetRxEventType(huart);
  switch (event_type)
  {
  case HAL_UART_RXEVENT_TC:
    if (g_rs232_rx_buf[1] == 0x03 && g_rs232_rx_buf[2] == 0x04)
    {
			uint16_t check = 0;
			check = crc16_modbus(0xFFFF,(const unsigned char*)g_rs232_rx_buf,7);
			if((check & 0x00FF) == g_rs232_rx_buf[7] && ((check >> 8) & 0x00FF) == g_rs232_rx_buf[8])
			{
				g_distance_uint32 = g_rs232_rx_buf[6];
				g_distance_uint32 |= (g_rs232_rx_buf[5] << 8);
				g_distance_uint32 |= (g_rs232_rx_buf[4] << 16);
				g_distance_uint32 |= (g_rs232_rx_buf[3] << 24);
				g_distance_f32 = g_distance_uint32 / 10.0F;
				HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
			}
    }
    g_rs232_status = 0;
    break;
  case HAL_UART_RXEVENT_HT:
    /* code */
    break;
  case HAL_UART_RXEVENT_IDLE:
    g_rs232_status = 0;
    break;

  default:
    break;
  }
}

void RS485_Status_Set(RS485_Channel ch, RS485_Status status)
{
  if (status == RS485_READ)
  {
    switch (ch)
    {
    case RS485_CH1:
      HAL_GPIO_WritePin(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RS485_1_RE_GPIO_Port, RS485_1_RE_Pin, GPIO_PIN_RESET);
      break;
    case RS485_CH2:
      HAL_GPIO_WritePin(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RS485_2_RE_GPIO_Port, RS485_2_RE_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
    }
  }
  else if (status == RS485_WRITE)
  {
    switch (ch)
    {
    case RS485_CH1:
      HAL_GPIO_WritePin(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RS485_1_RE_GPIO_Port, RS485_1_RE_Pin, GPIO_PIN_SET);
      break;
    case RS485_CH2:
      HAL_GPIO_WritePin(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RS485_2_RE_GPIO_Port, RS485_2_RE_Pin, GPIO_PIN_SET);
      break;

    default:
      break;
    }
  }
  else
  {
    return;
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
