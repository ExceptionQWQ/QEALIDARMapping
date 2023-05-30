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
#include "map.h"
#include "motion.h"
#include "detect_circle.h"

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
/* Definitions for lidarDecoder */
osThreadId_t lidarDecoderHandle;
const osThreadAttr_t lidarDecoder_attributes = {
  .name = "lidarDecoder",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mappingEngine */
osThreadId_t mappingEngineHandle;
const osThreadAttr_t mappingEngine_attributes = {
  .name = "mappingEngine",
  .stack_size = 700 * 4,
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
void MappingEngine(void *argument);

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

  /* creation of mappingEngine */
  mappingEngineHandle = osThreadNew(MappingEngine, NULL, &mappingEngine_attributes);

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
      snprintf(message, 64, "roll:%.2lf pitch:%.2lf heading:%.2lf\r\n", robotIMU.roll, robotIMU.pitch, robotIMU.heading);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源



//      //上传lidar数据
//      for (int i = 0; i < 360; ++i) {
//          snprintf(message, 64, "%d %d %d %d %d %.2lf\r\n", i, lidarPointData[i].distance, lidarPointData[i].intensity,
//                   lidarPointData[i].x, lidarPointData[i].y, lidarPointData[i].radian);
//          xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
//          HAL_UART_Transmit(&huart1, message, strlen(message), 100);
//          xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源
//      }



      snprintf(message, 64, "time_stamp:%d\r\n", lidar_time_stamp);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源


      osDelay(1000);
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
  lidarTaskHandle = xTaskGetCurrentTaskHandle(); //将当前任务句柄送给lidar

  /* Infinite loop */
  for(uint32_t cnt = 0; ; ++cnt)
  {
      if (HAL_UART_STATE_READY == HAL_UART_GetState(&huart3)) {
          HAL_UART_Receive_DMA(&huart3, lidarRecvBuff, 1024); //开启lidar串口通信
      }

      uint32_t value = 0;
      if (pdTRUE == xTaskNotifyWait(0, 0xffffffff, &value, pdMS_TO_TICKS(100)) && value == 0x12345678) {
          while (!DecodeLIDARPackage()) {} //解析lidar数据包
      }
      osDelay(1);
  }
  /* USER CODE END LidarDecoder */
}

/* USER CODE BEGIN Header_MappingEngine */
/**
* @brief Function implementing the mappingEngine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MappingEngine */
void MappingEngine(void *argument)
{
  /* USER CODE BEGIN MappingEngine */
  Init_Robot_Map();
  Init_Robot_Pos();
  /* Infinite loop */
  for(;;)
  {
      ClearLidarData();
    osDelay(1000);
      struct Detect_Circle_Result detectCircleResult = Ransac_Circles(105, 115);
      char message[64] = {0};
      snprintf(message, 64, "r:%d x:%d y:%d thresh:%d\r\n", detectCircleResult.radius, detectCircleResult.x, detectCircleResult.y, detectCircleResult.thresh);
      xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
      HAL_UART_Transmit(&huart1, message, strlen(message), 100);
      xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源

      if (detectCircleResult.radius && detectCircleResult.thresh > 15) {
          double k = atan((double)detectCircleResult.y / detectCircleResult.x);
          if (detectCircleResult.x > 0 && detectCircleResult.y > 0) {
              k += 0;
          } else if (detectCircleResult.x < 0 && detectCircleResult.y > 0) {
              k += PI;
          } else if (detectCircleResult.x < 0 && detectCircleResult.y < 0) {
              k += PI;
          } else {
              k += 2 * PI;
          }
          k += robotIMU.heading;
          if (k > 2 * PI) k -= 2 * PI;

          snprintf(message, 64, "heading:%.2lf spinto:%.2lf\r\n", robotIMU.heading, k);
          xSemaphoreTake(debugUartMutexHandle, portMAX_DELAY); //获取串口调试资源
          HAL_UART_Transmit(&huart1, message, strlen(message), 100);
          xSemaphoreGive(debugUartMutexHandle); //释放串口调试资源

          HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
          SpinTo(k);
          HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
          ClearSpeed();
          MoveForward(60);
          CommitSpeed();
          osDelay(1000);
      } else {
          struct NextLoc nextLoc = Find_Next_Loc();
          if (nextLoc.ret) {
              SpinTo(nextLoc.radian);
              ClearSpeed();
              MoveForward(60);
              CommitSpeed();
          } else {
              ClearSpeed();
              MoveForward(60);
              CommitSpeed();
          }
          osDelay(1000);
      }
      ClearSpeed();
      CommitSpeed();
  }
  /* USER CODE END MappingEngine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

