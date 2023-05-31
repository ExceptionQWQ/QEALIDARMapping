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

#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "queue.h"
#include "semphr.h"
#include "wheel_pwm.h"
#include "imu.h"
#include "motion.h"
#include "robot.h"

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuDecoder */
osThreadId_t imuDecoderHandle;
const osThreadAttr_t imuDecoder_attributes = {
  .name = "imuDecoder",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for robotCon */
osThreadId_t robotConHandle;
const osThreadAttr_t robotCon_attributes = {
  .name = "robotCon",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for debugUartMutex */
osMutexId_t debugUartMutexHandle;
const osMutexAttr_t debugUartMutex_attributes = {
  .name = "debugUartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMUDecoder(void *argument);
void RobotCon(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of debugUartMutex */
  debugUartMutexHandle = osMutexNew(&debugUartMutex_attributes);

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

  messageBufferHandle = xMessageBufferCreate(1024);

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuDecoder */
  imuDecoderHandle = osThreadNew(IMUDecoder, NULL, &imuDecoder_attributes);

  /* creation of robotCon */
  robotConHandle = osThreadNew(RobotCon, NULL, &robotCon_attributes);

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

      //上传左轮速度，pwm值
      char message[128] = {0};
//      snprintf(message, 64, "lSpeed:%.2lf lPwm:%.2lf\r\n", leftPWM.speed, leftPWM.pwm);
//      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
//      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
//      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
//      //上传右轮速度，pwm值
//      snprintf(message, 64, "rSpeed:%.2lf rPwm:%.2lf\r\n", rightPWM.speed, rightPWM.pwm);
//      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
//      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
//      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
//
      //上传imu数据
      snprintf(message, 64, "[imu] roll:%.2lf pitch:%.2lf heading:%.2lf\r\n", robotIMU.roll, robotIMU.pitch, robotIMU.heading);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源


//      //上传lidar数据
//      for (int i = 0; i < 360; ++i) {
//          snprintf(message, 64, "%d %d %d %d %d %.2lf\r\n", i, lidarPointData[i].distance, lidarPointData[i].intensity,
//                   lidarPointData[i].x, lidarPointData[i].y, lidarPointData[i].radian);
//          xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
//          HAL_UART_Transmit(&huart4, message, strlen(message), 100);
//          xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
//      }


//      snprintf(message, 64, "time_stamp:%d\r\n", lidar_time_stamp);
//      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
//      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
//      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源


      osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMUDecoder */
/**
* @brief Function implementing the imuDecoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUDecoder */
void IMUDecoder(void *argument)
{
  /* USER CODE BEGIN IMUDecoder */
  imuTaskHandle = xTaskGetCurrentTaskHandle(); //将当前任务句柄送给imu
  /* Infinite loop */
  for(uint32_t cnt = 0; ; ++cnt)
  {
      if (HAL_UART_STATE_READY == HAL_UART_GetState(&huart2)) {
          HAL_UART_Receive_DMA(&huart2, imuRecvBuff, 128); //开启imu串口通信
      }

      uint32_t value = 0;
      if (pdTRUE == xTaskNotifyWait(0, 0xffffffff, &value, pdMS_TO_TICKS(100)) && value == 0x12345678) {
          while (!DecodeIMUPackage()) {} //解析imu数据包
      }
      osDelay(1);
  }
  /* USER CODE END IMUDecoder */
}

/* USER CODE BEGIN Header_RobotCon */
/**
* @brief Function implementing the robotCon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotCon */
void RobotCon(void *argument)
{
  /* USER CODE BEGIN RobotCon */
    robotConTask = robotConHandle;
    HAL_UART_Receive_IT(&huart1, robotRecvBuff + robotRecvOffset, 1);
  /* Infinite loop */
  for(;;)
  {
    char message[128] = {0};
    xMessageBufferReceive(messageBufferHandle, message, 128, portMAX_DELAY);

      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源

    if (strstr(message, "[forward]") != NULL) {
        double speed;
        sscanf(message, "[forward] speed=%lf", &speed);
        ClearSpeed();
        MoveForward(speed);
        CommitSpeed();
    } else if (strstr(message, "[spin]") != NULL) {
        double radian;
        sscanf(message, "[spin] radian=%lf", &radian);
        SpinTo(radian);
    }
    osDelay(1);
  }
  /* USER CODE END RobotCon */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

