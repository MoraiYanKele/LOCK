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

#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include "oled.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum 
{
    INIT                    = 0x01,
    GET_IMAGE               = 0x02,          
    GEN_CHAR                = 0x03,
    SEARCH_FINGERPRINT      = 0x04,
    SUCCESS_USER            = 0x05,
    FAILURE                 = 0x06
} FingerprintStateTypedef;


typedef enum 
{

    ERR                    = 0x00,
    OK                     = 0x01,          

} FlagTypedef;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
TaskHandle_t xFingerprintModuleTaskHandle = NULL;
TaskHandle_t xOLEDTaskHandle = NULL;

FlagTypedef receiveFlag = ERR;
FingerprintStateTypedef fingerprintState;
uint8_t fingerID;

const uint8_t transmitBuff_PS_GetImage[20] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x01, 0x00, 0x05};
const uint8_t transmitBuff_PS_GenChar[20] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x04, 0x02, 0x01, 0x00, 0x08};
const uint8_t transmitBuff_PS_Search[20] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x08, 0x04, 0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x2E};
uint8_t receiveBuff[20];


const Image* imageArray[] = {
    
    &Expression_Blink_highImg,

    &Expression_HappyImg,
    &Expression_AngryImg,
    &Expression_AnnoyedImg,
    &Expression_Blink_lowImg,
    &Expression_BoredImg,
    &Expression_FocusedImg,
    &Expression_GleeImg,

    &Expression_Sad_upImg,

    &Expression_UnimpressedImg,

};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void FingerprintModuleTask(void *argument);
void OledTask(void *argument);

uint8_t SendCommand(const uint8_t* transmitBuff, uint16_t transmitSize, uint32_t timeout);
// 0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x03, 0x00, 0x00, 0x0A
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  printf("in callback\n");
  if (huart == &huart2)
  {
    // for (int i = 0; i < 15; i++)
    // {
    //   printf("0x%x", receiveBuff[i]);
    // }
    switch (fingerprintState)
    {
    case GET_IMAGE:
      if (receiveBuff[6] + receiveBuff[7] + receiveBuff[8] + receiveBuff[9] == receiveBuff[10] + receiveBuff[11])
      {
        if (receiveBuff[9] == 0x00)
        {
          receiveFlag = OK;
        }
        else 
        {
          receiveFlag = ERR;
        }
      }
      break;
    
    case GEN_CHAR:
      if (receiveBuff[6] + receiveBuff[7] + receiveBuff[8] + receiveBuff[9] == receiveBuff[10] + receiveBuff[11])
      {
        if (receiveBuff[9] == 0x00)
        {
          receiveFlag = OK;
        }
        else 
        {
          receiveFlag = ERR;
        }
      }
      break;
    
    case SEARCH_FINGERPRINT:
      if (receiveBuff[6] + receiveBuff[7] + receiveBuff[8] + receiveBuff[9] + receiveBuff[10] + receiveBuff[11] + receiveBuff[12] + receiveBuff[13] == receiveBuff[14] + receiveBuff[15])
      {
        if (receiveBuff[9] == 0x00)
        {
          receiveFlag = OK;
          fingerID = receiveBuff[10] + receiveBuff[11];
        } 
        else 
        {
          receiveFlag = ERR;
        }
      }
      break;
    default:
      break;
    }
    

  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBuff, sizeof(receiveBuff));
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(FingerprintModuleTask, "FingerprintRecognition", 256, NULL, osPriorityNormal, &xFingerprintModuleTaskHandle);
  // xTaskCreate(OledTask, "OLEDTask", 128, NULL, osPriorityNormal, &xOLEDTaskHandle);

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
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void FingerprintModuleTask(void *argument)
{
  uint8_t cmdResult = 0;
  fingerprintState = INIT;
  while (1)
  {
    switch (fingerprintState)
    {
    case INIT:
      fingerprintState = GET_IMAGE;
      printf("INIT\n");
      break;

    case GET_IMAGE:
      cmdResult = SendCommand(transmitBuff_PS_GetImage, 12, 3000);
      printf("GET_IMAGE\n");

      if (cmdResult == 1)
      {
        fingerprintState = GEN_CHAR;
      }
      else
      {
        fingerprintState = FAILURE;
      }
      break;

    case GEN_CHAR:
      cmdResult = SendCommand(transmitBuff_PS_GenChar, 13, 3000);
      printf("GEN_CHAR\n");

      if (cmdResult == 1)
      {
        fingerprintState = SEARCH_FINGERPRINT;
      }
      else
      {
        fingerprintState = FAILURE;
      }

      break;
    case SEARCH_FINGERPRINT:
      cmdResult = SendCommand(transmitBuff_PS_Search, 17, 3000);
      printf("SEARCH_FINGERPRINT\n");

      if (cmdResult == 1)
      {
        fingerprintState = SUCCESS_USER;
      }
      else
      {
        fingerprintState = FAILURE;
      }
      break;
    
    case SUCCESS_USER:
      printf("ok\n");
      // 打开门锁
      fingerprintState = INIT;
      break;

    case FAILURE:
      fingerprintState = INIT;
      break;


    default:
      break;
    }
  }
}

void OledTask(void *argument)
{
  vTaskDelay(20);
  OLED_Init();
 


//   HAL_Delay(500);
  // OLED_NewFrame();
  //      OLED_PrintString(0, 0, "波特律动hello", &font16x16, OLED_COLOR_NORMAL);

  // OLED_ShowFrame();
  while(1)
  {
    for (int i = 0; i < 10; i++)
    {
      OLED_NewFrame();
      OLED_DrawImage(0, 0, imageArray[i], OLED_COLOR_NORMAL); 
      OLED_ShowFrame();
      vTaskDelay(2000);
    }
    
  }
}

uint8_t SendCommand(const uint8_t* transmitBuff, uint16_t transmitSize, uint32_t timeout)
{
  printf("in sendcommend\n");
  uint32_t startTime = HAL_GetTick(); 
  HAL_UART_Transmit_DMA(&huart2, transmitBuff, transmitSize);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBuff, sizeof(receiveBuff));

  while (receiveFlag == ERR)
  {
    if ((HAL_GetTick() - startTime) > timeout)
    {
      receiveFlag = ERR;
      break;
    }
    osDelay(10);
  }

  if (receiveFlag == OK)
  {
    receiveFlag = ERR; 
    return 1;
  }
  else 
  {
    return 0;
  }
}
/* USER CODE END Application */

