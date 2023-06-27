/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SVPWM.h"
#include "Constants.h"

uint8_t data[2016];
uint16_t idx = 0;
uint16_t prev_uart_transmitted = 1;

uint16_t speedArray[16];
uint8_t speedArrayIdx = 0;
int16_t currentCount,delta_counts,previousCnt;
uint8_t speedArraynotFilled = 1;
uint16_t totalDeltaCount = 0;
uint16_t previousdeltaVal =  0;
uint16_t speed_s16=0;
uint16_t speed_rpm = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern RotorSpeedPos rtrSpdPosObj;
extern SVPWM svpwmObj;
extern uint8_t motorStart;
uint8_t cycleNo = 0;

uint16_t encoderIndexCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim2;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if (__HAL_GPIO_EXTI_GET_FLAG(ENC_INDEX_Pin))
		{
		  encoderIndexCounter++;
		  __HAL_TIM_SET_COUNTER(&htim2,0);
		}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENC_INDEX_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  prev_uart_transmitted += 1;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  //This interrupt fires at 50Khz, and the PWM is center Aligned.
  // so the interrupt fires at 0 CNT value and CNT value of TIM_PERIOD(1499).
  // We decide to only update the CCRs when the CNT value is at TIM PERIOD.
  //So that for a full CAPWM we have the same PWM value.(updating the PWM at 8KHZ not 16)
  uint16_t tim1_updateInterrupt = htim1.Instance-> CNT;
  if ( tim1_updateInterrupt >= 1300){
	  if (motorStart){
		  HAL_GPIO_WritePin(GPIOA,DAC_DBG_Pin,GPIO_PIN_SET);
		  update_RtrPos(&rtrSpdPosObj);
		  calculateSVPWM(&rtrSpdPosObj,&svpwmObj);
		  assign_SVPWM(&rtrSpdPosObj,&svpwmObj);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,svpwmObj.PhaseU);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,svpwmObj.PhaseV);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,svpwmObj.PhaseW);
		  HAL_GPIO_WritePin(GPIOA,DAC_DBG_Pin,GPIO_PIN_RESET);
/*
		  for (int idx=0;idx<8;i++){
			  data[idx*2] = (uint8_t)((data[idx] >> 8) & 0xff);
			  data[idx*2 + 1] = (uint8_t)(data[idx] & 0xff);
		  }
*/
		  //19 bits of data per cycle.and 2014 total data size. will fill up in 106 cycles.
		  /*if (idx < 2014){
			  data[idx] = (uint8_t)(cycleNo & 0xff);
			  data[idx+1] = (uint8_t)((rtrSpdPosObj.EAngle >> 8) & 0xff);
			  data[idx+2] = (uint8_t)(rtrSpdPosObj.EAngle & 0xff);
			  data[idx+3] = (uint8_t)(rtrSpdPosObj.sector & 0xff);  // (uint8)
			  data[idx+4] = (uint8_t)((rtrSpdPosObj.sectorAngle >> 8) & 0xff);
			  data[idx+5] = (uint8_t)(rtrSpdPosObj.sectorAngle & 0xff);
			  data[idx+6] = (uint8_t)(rtrSpdPosObj.currentCycle & 0xff); // (uint8)
			  data[idx+7] = (uint8_t)((svpwmObj.PV1 >> 8) & 0xff);
			  data[idx+8] = (uint8_t)((svpwmObj.PV1 ) & 0xff);
			  data[idx+9] = (uint8_t)((svpwmObj.PV2 >> 8) & 0xff);
			  data[idx+10] = (uint8_t)((svpwmObj.PV2 ) & 0xff);
			  data[idx+11] = (uint8_t)((svpwmObj.null >> 8) & 0xff);
			  data[idx+12] = (uint8_t)(svpwmObj.null & 0xff);
			  data[idx+13] = (uint8_t)((svpwmObj.PhaseU >> 8 ) & 0xff);
			  data[idx+14] = (uint8_t)((svpwmObj.PhaseU ) & 0xff);
			  data[idx+15] = (uint8_t)((svpwmObj.PhaseV >> 8 ) & 0xff);
			  data[idx+16] = (uint8_t)((svpwmObj.PhaseV ) & 0xff);
			  data[idx+17] = (uint8_t)(0x0A);//(svpwmObj.PhaseW >> 8 ) & 0xff);
			  data[idx+18] = (uint8_t)(0x0D);//((svpwmObj.PhaseW) & 0xff);
			  cycleNo++;
			  idx += 19;
		  }else{
			  if (prev_uart_transmitted){
				  HAL_UART_Transmit_DMA(&huart3,data,2014);
				  idx = 0;
				  cycleNo = 0;
				  prev_uart_transmitted = 0;
			  }
		  }*/

	  }
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  currentCount = htim2.Instance->CNT;
  //TODO : works only in the correct direction.
  delta_counts = currentCount - previousCnt;
  if (delta_counts < 0){
		delta_counts  = currentCount + (2048 - previousCnt);
  }
  previousCnt = currentCount;


  if (speedArraynotFilled){
	  speedArray[idx] = delta_counts;
	  totalDeltaCount += delta_counts;
  }else{
	  previousdeltaVal = speedArray[idx];
	  speedArray[idx] = delta_counts;
	  totalDeltaCount -= previousdeltaVal;
	  totalDeltaCount += delta_counts;
  }

  speed_s16 = totalDeltaCount * CNTS_TO_S16_CONSTANT;
  speed_rpm = totalDeltaCount * CNTS_TO_RPM_CONSTANT;
  idx++;

  rtrSpdPosObj.MSpd_RPM = speed_rpm;
  update_RtrSpd(&rtrSpdPosObj);

  if (idx >15){
	  idx = 0;
	  speedArraynotFilled = 0;
  }


  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
