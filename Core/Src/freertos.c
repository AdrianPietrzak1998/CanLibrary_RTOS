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
extern uint8_t testData[];
extern uint8_t testData2[256];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led1Task */
osThreadId_t Led1TaskHandle;
const osThreadAttr_t Led1Task_attributes = {
  .name = "Led1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Led2Task */
osThreadId_t Led2TaskHandle;
const osThreadAttr_t Led2Task_attributes = {
  .name = "Led2Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Led3Task */
osThreadId_t Led3TaskHandle;
const osThreadAttr_t Led3Task_attributes = {
  .name = "Led3Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPIOsemaphore */
osSemaphoreId_t GPIOsemaphoreHandle;
const osSemaphoreAttr_t GPIOsemaphore_attributes = {
  .name = "GPIOsemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Led1Toogle(void *argument);
void Led2Toogle(void *argument);
void Led3Toggle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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

  /* Create the semaphores(s) */
  /* creation of GPIOsemaphore */
  GPIOsemaphoreHandle = osSemaphoreNew(1, 0, &GPIOsemaphore_attributes);

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

  /* creation of Led1Task */
  Led1TaskHandle = osThreadNew(Led1Toogle, NULL, &Led1Task_attributes);

  /* creation of Led2Task */
  Led2TaskHandle = osThreadNew(Led2Toogle, NULL, &Led2Task_attributes);

  /* creation of Led3Task */
  Led3TaskHandle = osThreadNew(Led3Toggle, NULL, &Led3Task_attributes);

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
	  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(70);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Led1Toogle */
/**
* @brief Function implementing the Led1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led1Toogle */
void Led1Toogle(void *argument)
{
  /* USER CODE BEGIN Led1Toogle */
  /* Infinite loop */
  for(;;)
  {
	  Can20SendLongMsgCrc(&hcan1, 0x100, testData, 16);
	  Can20SendLongMsgCrc(&hcan2, 0xffff, testData2, 256);
    osDelay(200);
  }
  /* USER CODE END Led1Toogle */
}

/* USER CODE BEGIN Header_Led2Toogle */
/**
* @brief Function implementing the Led2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led2Toogle */
void Led2Toogle(void *argument)
{
  /* USER CODE BEGIN Led2Toogle */
  /* Infinite loop */
  for(;;)
  {
	  Can20SendCheck(&hcan1, 0x015);
	  Can20SendCheck(&hcan1, 0x010);
    osDelay(500);
  }
  /* USER CODE END Led2Toogle */
}

/* USER CODE BEGIN Header_Led3Toggle */
/**
* @brief Function implementing the Led3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led3Toggle */
void Led3Toggle(void *argument)
{
  /* USER CODE BEGIN Led3Toggle */
  /* Infinite loop */
  for(;;)
  {
	  Can20SendSimple(&hcan1, 0x200, testData, 8);
	  Can20SendCheck(&hcan3, 0xfffa);
    osDelay(1100);
  }
  /* USER CODE END Led3Toggle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

