/*
 * EncoderCalibration.c
 *
 *  Created on: Feb 18, 2023
 *      Author: harsha
 */

#include "stm32g4xx_hal.h"
#include "EncoderCalibration.h"

uint16_t encCalib_delayTime = 0;
uint16_t encCalib_pwmVal = 0;

extern TIM_HandleTypeDef htim1;


void setupCalibration(uint8_t dutyPercentage,uint16_t delayTime_ms){
	encCalib_pwmVal = ((float)dutyPercentage/100.0) * htim1.Instance->ARR;
	encCalib_delayTime = delayTime_ms;
}

void setPhaseU(uint16_t pwmVal){
	htim1.Instance->CCR1 = pwmVal;
}
void setPhaseV(uint16_t pwmVal){
	htim1.Instance->CCR2 = pwmVal;
}
void setPhaseW(uint16_t pwmVal){
	htim1.Instance->CCR3 = pwmVal;
}

void TurnOffAllPhases(void){
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
}

void voltageOnPrincipalAxis(uint8_t axis){
	if (axis == 1){
		setPhaseU(encCalib_pwmVal);
		setPhaseV(0);
		setPhaseW(0);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
	if (axis == 2){
		setPhaseU(encCalib_pwmVal);
		setPhaseV(encCalib_pwmVal);
		setPhaseW(0);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
	if (axis == 3){
		setPhaseU(0);
		setPhaseV(encCalib_pwmVal);
		setPhaseW(0);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
	if (axis == 4){
		setPhaseU(0);
		setPhaseV(encCalib_pwmVal);
		setPhaseW(encCalib_pwmVal);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
	if (axis == 5){
		setPhaseU(0);
		setPhaseV(0);
		setPhaseW(encCalib_pwmVal);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
	if (axis == 6){
		setPhaseU(encCalib_pwmVal);
		setPhaseV(0);
		setPhaseW(encCalib_pwmVal);
		HAL_Delay(encCalib_delayTime);
		TurnOffAllPhases();
	}
}
