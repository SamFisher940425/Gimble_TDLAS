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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask_WDog */
osThreadId_t myTask_WDogHandle;
const osThreadAttr_t myTask_WDog_attributes = {
  .name = "myTask_WDog",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask_232Rx */
osThreadId_t myTask_232RxHandle;
const osThreadAttr_t myTask_232Rx_attributes = {
  .name = "myTask_232Rx",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask_485C1Tx */
osThreadId_t myTask_485C1TxHandle;
const osThreadAttr_t myTask_485C1Tx_attributes = {
  .name = "myTask_485C1Tx",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask_WorkFlow */
osThreadId_t myTask_WorkFlowHandle;
const osThreadAttr_t myTask_WorkFlow_attributes = {
  .name = "myTask_WorkFlow",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask_WDog(void *argument);
void StartTask_232Rx(void *argument);
void StartTask_485C1Tx(void *argument);
void StartTask_WorkFlow(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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

  /* creation of myTask_485C1Tx */
  myTask_485C1TxHandle = osThreadNew(StartTask_485C1Tx, NULL, &myTask_485C1Tx_attributes);

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
  for(;;)
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
  for(;;)
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
  for(;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);
  }
  /* USER CODE END StartTask_232Rx */
}

/* USER CODE BEGIN Header_StartTask_485C1Tx */
/**
* @brief Function implementing the myTask_485C1Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_485C1Tx */
void StartTask_485C1Tx(void *argument)
{
  /* USER CODE BEGIN StartTask_485C1Tx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_485C1Tx */
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_WorkFlow */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

