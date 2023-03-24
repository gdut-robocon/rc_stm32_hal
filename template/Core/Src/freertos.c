/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "moto.h"
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for move_base_task */
osThreadId_t move_base_taskHandle;
const osThreadAttr_t move_base_task_attributes = {
  .name = "move_base_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Robot_state_tas */
osThreadId_t Robot_state_tasHandle;
const osThreadAttr_t Robot_state_tas_attributes = {
  .name = "Robot_state_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Auto_task */
osThreadId_t Auto_taskHandle;
const osThreadAttr_t Auto_task_attributes = {
  .name = "Auto_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for data_update_tas */
osThreadId_t data_update_tasHandle;
const osThreadAttr_t data_update_tas_attributes = {
  .name = "data_update_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for motor_control_t */
osThreadId_t motor_control_tHandle;
const osThreadAttr_t motor_control_t_attributes = {
  .name = "motor_control_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void move_base(void *argument);
void Robot_state(void *argument);
void Auto(void *argument);
void data_update(void *argument);
void motor_control(void *argument);

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

  /* creation of move_base_task */
  move_base_taskHandle = osThreadNew(move_base, NULL, &move_base_task_attributes);

  /* creation of Robot_state_tas */
  Robot_state_tasHandle = osThreadNew(Robot_state, NULL, &Robot_state_tas_attributes);

  /* creation of Auto_task */
  Auto_taskHandle = osThreadNew(Auto, NULL, &Auto_task_attributes);

  /* creation of data_update_tas */
  data_update_tasHandle = osThreadNew(data_update, NULL, &data_update_tas_attributes);

  /* creation of motor_control_t */
  motor_control_tHandle = osThreadNew(motor_control, NULL, &motor_control_t_attributes);

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
		uint16_t speedtest=0; 
		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT=speedtest;

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_move_base */
/**
* @brief Function implementing the move_base_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_move_base */
void move_base(void *argument)
{
  /* USER CODE BEGIN move_base */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END move_base */
}

/* USER CODE BEGIN Header_Robot_state */
/**
* @brief Function implementing the Robot_state_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_state */
void Robot_state(void *argument)
{
  /* USER CODE BEGIN Robot_state */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Robot_state */
}

/* USER CODE BEGIN Header_Auto */
/**
* @brief Function implementing the Auto_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Auto */
void Auto(void *argument)
{
  /* USER CODE BEGIN Auto */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Auto */
}

/* USER CODE BEGIN Header_data_update */
/**
* @brief Function implementing the data_update_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_update */
void data_update(void *argument)
{
  /* USER CODE BEGIN data_update */
  /* Infinite loop */
  for(;;)
  {
		chassis_m3508_send_motor_currents();
		
	
    osDelay(5);
  }
  /* USER CODE END data_update */
}

/* USER CODE BEGIN Header_motor_control */
/**
* @brief Function implementing the motor_control_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_control */
void motor_control(void *argument)
{
  /* USER CODE BEGIN motor_control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
