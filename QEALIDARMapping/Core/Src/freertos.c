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
#include "lidar.h"

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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuDecoder */
osThreadId_t imuDecoderHandle;
const osThreadAttr_t imuDecoder_attributes = {
  .name = "imuDecoder",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lidarDecoder */
osThreadId_t lidarDecoderHandle;
const osThreadAttr_t lidarDecoder_attributes = {
  .name = "lidarDecoder",
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
void LidarDecoder(void *argument);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuDecoder */
  imuDecoderHandle = osThreadNew(IMUDecoder, NULL, &imuDecoder_attributes);

  /* creation of lidarDecoder */
  lidarDecoderHandle = osThreadNew(LidarDecoder, NULL, &lidarDecoder_attributes);

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
      char message[64] = {0};
      snprintf(message, 64, "lSpeed:%.2lf lPwm:%.2lf\r\n", leftPWM.speed, leftPWM.pwm);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
      //上传右轮速度，pwm值
      snprintf(message, 64, "rSpeed:%.2lf rPwm:%.2lf\r\n", rightPWM.speed, rightPWM.pwm);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源

      //上传imu数据
      snprintf(message, 64, "roll:%.2lf pitch:%.2lf heading:%.2lf\r\n", robotIMU.roll, robotIMU.pitch, robotIMU.heading);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源


      //上传lidar数据
      for (int i = 0; i < 360; ++i) {
          snprintf(message, 64, "%d %d %d\r\n", i, lidarPointData[i].distance, lidarPointData[i].intensity);
          xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
          HAL_UART_Transmit(&huart1, message, strlen(message), 100);
          xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
      }

      snprintf(message, 64, "time_stamp:%d\r\n", lidar_time_stamp);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源

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
  HAL_UART_Receive_IT(&huart2, imuRecvBuff1, 1); //开启imu串口通信
  /* Infinite loop */
  for(uint32_t cnt = 0; ; ++cnt)
  {
      uint32_t value = 0;
      if (pdTRUE == xTaskNotifyWait(0, 0xffffffff, &value, portMAX_DELAY) && value == 0x12345678) {
          DecodeIMUPackage(); //解析imu数据包
      }
      osDelay(1);
  }
  /* USER CODE END IMUDecoder */
}

/* USER CODE BEGIN Header_LidarDecoder */
/**
* @brief Function implementing the lidarDecoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LidarDecoder */
void LidarDecoder(void *argument)
{
  /* USER CODE BEGIN LidarDecoder */
  debugUartMutex = debugUartMutexHandle;
  lidarDecoderHandle = xTaskGetCurrentTaskHandle(); //将当前任务句柄送给lidar
  HAL_UART_Receive_IT(&huart3, lidarRecvBuff1, 1); //开启lidar串口通信

  /* Infinite loop */
  for(uint32_t cnt = 0; ; ++cnt)
  {
      //不知道为什么串口3的抢占优先级必须小于5
      while (1) {
          int ret = DecodeLIDARPackage();
          if (ret == 0) break;
      }
      osDelay(1);
  }
  /* USER CODE END LidarDecoder */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

