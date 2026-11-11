/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "motor_task.h"
#include "daemon.h"
#include "robot.h"
#include "ins.h"
#include "chassis_log.h"
#include "config.h"
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
osThreadId ledTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId insTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartMOTORTASK(void const *argument);

void StartDAEMONTASK(void const *argument);

void StartROBOTTASK(void const *argument);

void StartINSTASK(void const * argument);
static void StartMotionLogTask(void const *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadDef(motortask, StartMOTORTASK, (osPriority)g_task_config.motor.priority, 0, g_task_config.motor.stack_words);
  motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

  osThreadDef(daemontask, StartDAEMONTASK, (osPriority)g_task_config.daemon.priority, 0, g_task_config.daemon.stack_words);
  daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

  osThreadDef(robottask, StartROBOTTASK, (osPriority)g_task_config.robot.priority, 0, g_task_config.robot.stack_words);
  defaultTaskHandle = osThreadCreate(osThread(robottask), NULL);

  osThreadDef(instask, StartINSTASK, (osPriority)g_task_config.ins.priority, 0, g_task_config.ins.stack_words);
  insTaskHandle = osThreadCreate(osThread(instask), NULL);
  /* 文本格式化含整数 snprintf 调用链，日志栈扩至 512 words（2 KiB）。 */
  osThreadDef(motionlog, StartMotionLogTask, (osPriority)g_task_config.motion_log.priority, 0, g_task_config.motion_log.stack_words);
  osThreadCreate(osThread(motionlog), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  vTaskDelete( NULL );
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


void StartMOTORTASK(void const *argument)
{
  while (1)
  {
    // 500Hz
    MotorControlTask();
    osDelay(g_task_config.motor.period_ticks);
  }
}

void StartDAEMONTASK(void const *argument)
{
  while (1)
  {
    // 100Hz
    DaemonTask();
    osDelay(g_task_config.daemon.period_ticks);
  }
}

void StartROBOTTASK(void const *argument)
{
  while (1)
  {
    // 200Hz
    RobotTask();
    osDelay(g_task_config.robot.period_ticks);
  }
}

void StartINSTASK(void const * argument)
{
  while (1)
  {
    // 200Hz
    INS_Task();
    osDelay(g_task_config.ins.period_ticks);
  }
}

static void StartMotionLogTask(void const *argument)
{
  (void)argument;
  for (;;)
  {
    ChassisMotionLogTask();
    osDelay(g_task_config.motion_log.period_ticks);
  }
}
/* USER CODE END Application */
