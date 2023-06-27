/*
 * EncoderCalibration.h
 *
 *  Created on: Feb 18, 2023
 *      Author: harsha
 */

#ifndef ENCODERCALIBRATION_H_
#define ENCODERCALIBRATION_H_

void setupCalibration(uint8_t dutyPercentage,uint16_t delayTime_ms);
void setPhaseU(uint16_t pwmVal);
void setPhaseV(uint16_t pwmVal);
void setPhaseW(uint16_t pwmVal);
void TurnOffAllPhases(void);
void voltageOnPrincipalAxis(uint8_t axis);

#endif /* ENCODERCALIBRATION_H_ */
