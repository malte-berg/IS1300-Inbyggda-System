/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "Traffic.h"
#include "event_groups.h"
#include "Test.h"
#include "traffic_functions.h"
#include "semphr.h"
#include "oled_functions.h"
#include "adc.h"
#include "tim.h"

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

EventGroupHandle_t eventGroup;
EventGroupHandle_t oledEventGroup;
uint32_t current_instruction;
bool doBlink1, doBlink2;
bool blinkState;
SemaphoreHandle_t lightMutex;
SemaphoreHandle_t oledMutex;
oledBarStruct oledBars[NUM_BARS];
uint16_t poti_value;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TLHandlerTask */
osThreadId_t TLHandlerTaskHandle;
const osThreadAttr_t TLHandlerTask_attributes = {
  .name = "TLHandlerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PLHandlerTask */
osThreadId_t PLHandlerTaskHandle;
const osThreadAttr_t PLHandlerTask_attributes = {
  .name = "PLHandlerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InHandlerTask */
osThreadId_t InHandlerTaskHandle;
const osThreadAttr_t InHandlerTask_attributes = {
  .name = "InHandlerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for blinkTaskHandle */
osThreadId_t blinkTaskHandleHandle;
const osThreadAttr_t blinkTaskHandle_attributes = {
  .name = "blinkTaskHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLHandlerTask */
osThreadId_t OLHandlerTaskHandle;
const osThreadAttr_t OLHandlerTask_attributes = {
  .name = "OLHandlerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TLHandler(void *argument);
void PLHandler(void *argument);
void InHandler(void *argument);
void blinkTask(void *argument);
void OLHandler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  eventGroup = xEventGroupCreate();
  oledEventGroup = xEventGroupCreate();
  doBlink1 = false;
  doBlink2 = false;
  blinkState = false;
  

  HAL_GPIO_WritePin(SR_Reset_GPIO_Port, SR_Reset_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(SR_Enable_GPIO_Port, SR_Enable_Pin, GPIO_PIN_RESET);

  //init bars
  for(int i = 0; i < NUM_BARS; i++) {
      oledBars[i].startTick = 0;
      oledBars[i].durationTick = 0;
      oledBars[i].active = 0;
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  lightMutex = xSemaphoreCreateMutex();
  oledMutex = xSemaphoreCreateMutex();

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

  /* creation of TLHandlerTask */
  TLHandlerTaskHandle = osThreadNew(TLHandler, NULL, &TLHandlerTask_attributes);

  /* creation of PLHandlerTask */
  PLHandlerTaskHandle = osThreadNew(PLHandler, NULL, &PLHandlerTask_attributes);

  /* creation of InHandlerTask */
  InHandlerTaskHandle = osThreadNew(InHandler, NULL, &InHandlerTask_attributes);

  /* creation of blinkTaskHandle */
  blinkTaskHandleHandle = osThreadNew(blinkTask, NULL, &blinkTaskHandle_attributes);

  /* creation of OLHandlerTask */
  OLHandlerTaskHandle = osThreadNew(OLHandler, NULL, &OLHandlerTask_attributes);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TLHandler */
/**
* @brief Function implementing the TLHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TLHandler */
void TLHandler(void *argument)
{
  /* USER CODE BEGIN TLHandler */
  /* Infinite loop */
	static states State, NextState;
  for(;;)
  {
    State = NSG_EWR;
    NextState = NSG_EWR;
    uint32_t instruction, elapsedTime, elapsedTime2;
    bool toGreen = false, firstRound  = true; // decide whether to go from yellow to red or yellow to green 
    EventBits_t receivedBits;
    TickType_t xStartTimer, xEndTimer, timeToDelay;
    instruction = PL1_Green | PL2_Red;
    current_instruction = update_instruction(current_instruction, instruction, PL);
    while (1) {
        State = NextState;
        switch(State) {
        case NSG_EWR:
            instruction = TL_NS_Green | TL_EW_Red;
            current_instruction = update_instruction(current_instruction, instruction, TL);
            startBar(G1, greenDelay);
            if (firstRound) {
              startBar(W1, walkingDelay);
            }
            stopBar(R1);
            // check if the PL2 is pressed, or there is a car by TL1 or TL3
            xStartTimer = xTaskGetTickCount();
            receivedBits = xEventGroupWaitBits(eventGroup, Event_PL2 | Event_TL1_Switch | Event_TL3_Switch | Event_TL2_Switch | Event_TL4_Switch | Event_PL2_Pressed_Yellow, pdTRUE, pdFALSE, greenDelay);
            xEndTimer = xTaskGetTickCount();
            elapsedTime = xEndTimer - xStartTimer;
            if(receivedBits & Event_PL2 || receivedBits & Event_PL2_Pressed_Yellow) {
              //doBlink2 = true;
              //startBar(P2, pedestrianDelay);
              timeToDelay = greenDelay - elapsedTime < pedestrianDelay ? (greenDelay - elapsedTime) : pedestrianDelay - yellowDelay;
              if (elapsedTime < walkingDelay && firstRound) timeToDelay = walkingDelay - elapsedTime;
              vTaskDelay(timeToDelay);
            } else if((receivedBits & Event_TL2_Switch || receivedBits & Event_TL4_Switch) && !(receivedBits & Event_TL1_Switch || receivedBits & Event_TL3_Switch)) {
              NextState = NSG_EWR;
              if (elapsedTime > walkingDelay) stopBar(W1);
              firstRound = false;
              break;
            } else if(receivedBits & Event_TL1_Switch || receivedBits & Event_TL3_Switch) {
              if(!(receivedBits & Event_TL2_Switch || receivedBits & Event_TL4_Switch)) {
                if(!firstRound) {
                  toGreen = false;
                  NextState = NSY_EWR;
                  startBar(R2, redDelayMax);
                  xStartTimer = xTaskGetTickCount();
                  receivedBits = xEventGroupWaitBits(eventGroup, Event_PL2, pdFALSE, pdFALSE, redDelayMax);
                  if (receivedBits & Event_PL2) {
                    xEndTimer = xTaskGetTickCount();
                    elapsedTime = xEndTimer - xStartTimer;
                    //doBlink2 = true;
                    startBar(P2, pedestrianDelay);
                    vTaskDelay(redDelayMax - elapsedTime);
                  }
                  break;
                } 
              }
              startBar(R2, redDelayMax);
              stopBar(G1);
              timeToDelay = greenDelay - elapsedTime < redDelayMax ? (greenDelay - elapsedTime) : redDelayMax;
              if (elapsedTime < walkingDelay && firstRound) timeToDelay = walkingDelay - elapsedTime;
              vTaskDelay(timeToDelay);
            }
            firstRound = false;
            NextState = NSY_EWR;
            toGreen = false;
            break;
        case NSY_EWR:
            instruction = TL_NS_Yellow | TL_EW_Red;
            current_instruction = update_instruction(current_instruction, instruction, TL);
            if (toGreen) {
              startBar(R2, redDelayMax);
              stopBar(G2);
              xStartTimer = xTaskGetTickCount();
              receivedBits = xEventGroupWaitBits(eventGroup, Event_PL2, pdTRUE, pdFALSE, yellowDelay);
              xEndTimer = xTaskGetTickCount();
              elapsedTime = xEndTimer - xStartTimer;
              if(receivedBits & Event_PL2) {
                //doBlink2 = true;
                xEventGroupSetBits(eventGroup, Event_PL2_Pressed_Yellow);
                vTaskDelay(yellowDelay - elapsedTime);
              }
              NextState = NSG_EWR;
            } else {
              vTaskDelay(yellowDelay);
              NextState = NSR_EWY;
              xEventGroupSetBits(eventGroup, Event_EW_Safe_Walk);
              doBlink2 = false;
              current_instruction = update_instruction(current_instruction, 0, PLB);
            }
            toGreen = true;
            firstRound = true;
            break;
        case NSR_EWY:
            instruction = TL_NS_Red | TL_EW_Yellow;
            current_instruction = update_instruction(current_instruction, instruction, TL);
            if (toGreen) {
              stopBar(G1);
              xStartTimer = xTaskGetTickCount();
              receivedBits = xEventGroupWaitBits(eventGroup, Event_PL1, pdTRUE, pdFALSE, yellowDelay);
              xEndTimer = xTaskGetTickCount();
              elapsedTime = xEndTimer - xStartTimer;
              if(receivedBits & Event_PL1) {
                //doBlink1 = true;
                xEventGroupSetBits(eventGroup, Event_PL1_Pressed_Yellow);
                vTaskDelay(yellowDelay - elapsedTime);
              }
              NextState = NSR_EWG;
            } else {
              vTaskDelay(yellowDelay);
              NextState = NSY_EWR;
              xEventGroupSetBits(eventGroup, Event_NS_Safe_Walk); // tell the pedestrian light that the N/S traffic is green
              doBlink1 = false;
              current_instruction = update_instruction(current_instruction, 0, PLB);
            }
            toGreen = true;
            firstRound = true;
            break;
        case NSR_EWG:
            instruction = TL_NS_Red | TL_EW_Green;
            current_instruction = update_instruction(current_instruction, instruction, TL);
            startBar(G2, greenDelay);
            if (firstRound) {
              startBar(W2, walkingDelay);
            }
            stopBar(R2);
            xStartTimer = xTaskGetTickCount();
            receivedBits = xEventGroupWaitBits(eventGroup, Event_PL1 | Event_TL1_Switch | Event_TL3_Switch | Event_TL2_Switch | Event_TL4_Switch | Event_PL1_Pressed_Yellow, pdTRUE, pdFALSE, greenDelay);
            xEndTimer = xTaskGetTickCount();
            elapsedTime = xEndTimer - xStartTimer;

            if(receivedBits & Event_PL1 || receivedBits & Event_PL1_Pressed_Yellow) {
              //doBlink1 = true;
              startBar(P1, pedestrianDelay);
              timeToDelay = greenDelay - elapsedTime < pedestrianDelay ? (greenDelay - elapsedTime) : pedestrianDelay - yellowDelay;
              if (elapsedTime < walkingDelay && firstRound) timeToDelay = walkingDelay - elapsedTime;
              vTaskDelay(timeToDelay);
            } else if((receivedBits & Event_TL1_Switch || receivedBits & Event_TL3_Switch) && !(receivedBits & Event_TL2_Switch || receivedBits & Event_TL4_Switch)) {
              NextState = NSR_EWG;
              firstRound = false;
              break;
            } else if(receivedBits & Event_TL2_Switch || receivedBits & Event_TL4_Switch ) {
              if(!(receivedBits & Event_TL1_Switch || receivedBits & Event_TL3_Switch)) {
                if(!firstRound) {
                  toGreen = false;
                  NextState = NSR_EWY;
                  startBar(R1, redDelayMax);
                  xStartTimer = xTaskGetTickCount();
                  receivedBits = xEventGroupWaitBits(eventGroup, Event_PL1, pdFALSE, pdFALSE, redDelayMax);
                  if (receivedBits & Event_PL1) {
                    xEndTimer = xTaskGetTickCount();
                    elapsedTime = xEndTimer - xStartTimer;
                    //doBlink1 = true;
                    startBar(P1, pedestrianDelay);
                    vTaskDelay(redDelayMax - elapsedTime);
                  }
                  break;
                }
              }
              startBar(R1, redDelayMax);
              stopBar(G2);
              timeToDelay = greenDelay - elapsedTime < redDelayMax ? (greenDelay - elapsedTime) : redDelayMax;
              if (elapsedTime < walkingDelay && firstRound) timeToDelay = walkingDelay - elapsedTime;
              vTaskDelay(timeToDelay);
            }
            toGreen = false;
            firstRound = false;
            NextState = NSR_EWY;
            break;
        }
      }
      vTaskDelay(10);
  }
  /* USER CODE END TLHandler */
}

/* USER CODE BEGIN Header_PLHandler */
/**
* @brief Function implementing the PLHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PLHandler */
void PLHandler(void *argument)
{
  /* USER CODE BEGIN PLHandler */

  /* Infinite loop */
  EventBits_t receivedBits;
  uint32_t instruction;
  for(;;)
  {

	  receivedBits = xEventGroupWaitBits(eventGroup, Event_NS_Safe_Walk  | Event_EW_Safe_Walk, pdFALSE, pdFALSE, portMAX_DELAY);
      if(receivedBits & Event_NS_Safe_Walk) {
    	  instruction = PL1_Green | PL2_Red;
    	  current_instruction = update_instruction(current_instruction, instruction, PL);
        stopBar(P1);
        vTaskDelay(walkingDelay);
        xEventGroupClearBits(eventGroup, Event_NS_Safe_Walk);
      }
      if(receivedBits & Event_EW_Safe_Walk) {
        instruction = PL1_Red | PL2_Green;
    	  current_instruction = update_instruction(current_instruction, instruction, PL);
        stopBar(P2);
        vTaskDelay(walkingDelay);
        xEventGroupClearBits(eventGroup, Event_EW_Safe_Walk);
      }

  }
  /* USER CODE END PLHandler */
}

/* USER CODE BEGIN Header_InHandler */
/**
* @brief Function implementing the InHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InHandler */
void InHandler(void *argument)
{
  /* USER CODE BEGIN InHandler */
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GPIO_ReadPin(PL2_Switch_GPIO_Port, PL2_Switch_Pin) == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_PL2);
    //add a mutex 
      if(current_instruction & PL2_Red) {
        if(!doBlink2) startBar(P2, pedestrianDelay);
        doBlink2 = true;
      }
	  } else {
      xEventGroupClearBits(eventGroup, Event_PL2);
    }
    if(HAL_GPIO_ReadPin(PL1_Switch_GPIO_Port, PL1_Switch_Pin) == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_PL1);
      if(current_instruction & PL1_Red) {
        if (!doBlink1) startBar(P1, pedestrianDelay);
        doBlink1 = true;
      }
    } else {
      xEventGroupClearBits(eventGroup, Event_PL1);
    }
    if(HAL_GPIO_ReadPin(TL1_Car_GPIO_Port,TL1_Car_Pin) == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_TL1_Switch);
    } else {
      xEventGroupClearBits(eventGroup, Event_TL1_Switch);
    }
    if(HAL_GPIO_ReadPin(TL2_Car_GPIO_Port,TL2_Car_Pin) == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_TL2_Switch);
    } else {
      xEventGroupClearBits(eventGroup, Event_TL2_Switch);
    }
    if(HAL_GPIO_ReadPin(TL3_Car_GPIO_Port,TL3_Car_Pin) == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_TL3_Switch);
    } else {
      xEventGroupClearBits(eventGroup, Event_TL3_Switch);
    }
    if(HAL_GPIO_ReadPin(TL4_Car_GPIO_Port,TL4_Car_Pin)  == GPIO_PIN_RESET) {
      xEventGroupSetBits(eventGroup, Event_TL4_Switch);
    } else {
      xEventGroupClearBits(eventGroup, Event_TL4_Switch);
    }
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    poti_value = HAL_ADC_GetValue(&hadc1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, poti_value);
	vTaskDelay(10);
  }
  /* USER CODE END InHandler */
}

/* USER CODE BEGIN Header_blinkTask */
/**
* @brief Function implementing the blinkTaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blinkTask */
void blinkTask(void *argument)
{
  /* USER CODE BEGIN blinkTask */
  /* Infinite loop */
  for(;;)
  {
    if (doBlink1 || doBlink2) {
      uint32_t instruction = 0x0;
      if (doBlink1) instruction |= blinkState ? PL1_Blue : 0;
      if (doBlink2) instruction |= blinkState ? PL2_Blue : 0;
      current_instruction = update_instruction(current_instruction, instruction, PLB);
    }
    blinkState = !blinkState;
    vTaskDelay(toggleFreq);
  }
  /* USER CODE END blinkTask */
}

/* USER CODE BEGIN Header_OLHandler */
/**
* @brief Function implementing the OLHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLHandler */
void OLHandler(void *argument)
{
  /* USER CODE BEGIN OLHandler */
  /* Infinite loop */
  uint8_t screenArray[NUM_BARS];
  drawTitles();
  drawBarOutlines();
  
  for(;;)
  {
    TickType_t current = xTaskGetTickCount();
    xSemaphoreTake(oledMutex, portMAX_DELAY);
    for(int i = 0; i < NUM_BARS; i++) {
      if(oledBars[i].active == 0) {
        screenArray[i] = BAR_Y1; // bar empty
        continue;
      }

      TickType_t start  = oledBars[i].startTick;
      TickType_t total  = oledBars[i].durationTick;
      TickType_t elapsed = current - start;

      if(elapsed >= total) {
        oledBars[i].active = 0;
        screenArray[i] = BAR_Y1;
        continue;
      }

      float barRatio = 1.0f - ((float)elapsed / (float)total);
      if(barRatio < 0.0f) barRatio = 0.0f;
      if(barRatio > 1.0f) barRatio = 1.0f;

      uint8_t top = BAR_Y1 - (uint8_t)((barRatio * MAX_BAR_HEIGHT));

      if(top < BAR_Y2) top = BAR_Y2;
      if(top > BAR_Y1) top = BAR_Y1;

      screenArray[i] = top;
    }
    xSemaphoreGive(oledMutex);
    drawBars(screenArray);
    osDelay(20);
  }
  /* USER CODE END OLHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

