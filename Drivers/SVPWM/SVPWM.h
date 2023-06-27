/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SVPWM_H
#define __SVPWM_H

#include <stdint.h>
#include <stm32g4xx_hal.h>
#include "libmathq15.h"

#define SIXTY_DEG_Q16ANGLE 10922

typedef struct rotorSpdPosStruct {
  q16angle_t MAngle; //0-359.99 = > 0-65535
  q16angle_t EAngle; // 0-359.99 => 0-65535
  uint16_t EPos_dpp;
  uint16_t MPos_dpp;
  q16angle_t sectorAngle; // 0-59.99=> 0-16383
  uint8_t sector;  // 0-5
  uint16_t totalsectorCycles;
  uint16_t currentCycle;

  uint16_t MSpd_RPM;
  uint16_t MSpd_s16;
}RotorSpeedPos;

typedef struct SVPWM_Struct{
  uint16_t CFG_maxModulationIndex; 
  uint16_t CFG_timerCntMax;
  q15_t modulationIndex; // without THI it becomes 0-0.866
  uint16_t PV1;
  uint16_t PV2;
  uint16_t null;
  uint16_t CNTA;
  uint16_t CNTB;
  uint16_t CNTC;
  uint16_t PhaseU;
  uint16_t PhaseV;
  uint16_t PhaseW;
}SVPWM;

void calculateSVPWM(RotorSpeedPos *rtrSpdPos, SVPWM *svpwm);
void initSVPWM(SVPWM *s);
void init_RtrSpdPos(RotorSpeedPos *rtrSpdPos,uint16_t startingAngle);
void update_RtrPos(RotorSpeedPos *rtrSpdPos);
void update_RtrSpd(RotorSpeedPos *rtrSpdPos);
void assign_SVPWM(RotorSpeedPos *rtrSpdPos, SVPWM *svpwm);
#endif
