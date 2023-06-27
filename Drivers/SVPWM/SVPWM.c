#include <SVPWM.h>
#include <Constants.h>

void initSVPWM(SVPWM *s){
  s->CFG_maxModulationIndex =28377; //0.866 without THI ( 32768 is 1)
  s->CFG_timerCntMax = 1499;
  s->modulationIndex = 3000; // is approx 10%
  s->null = 0;
  s->PV1 = 0;
  s->PV2 = 0;
}

void init_RtrSpdPos(RotorSpeedPos *rtrSpdPos,uint16_t startingAngle){
  //sector angle is in q16Angle_t and sector is uint8_t
  rtrSpdPos->MAngle = startingAngle * CNTS_TO_MANGLE_S16;
  rtrSpdPos->EAngle = (rtrSpdPos->MAngle % MANGLE_TO_EANGLE_MOD_DIVIDEND) * MANGLE_TO_EANGLE_MULTIPLIER;
  rtrSpdPos->sector = rtrSpdPos->EAngle/SIXTY_DEG_Q16ANGLE; //  goes from 0-5
  rtrSpdPos->sectorAngle = rtrSpdPos->EAngle - (rtrSpdPos->sector*SIXTY_DEG_Q16ANGLE);
}

void update_RtrSpd(RotorSpeedPos *rtrSpdPos){
  rtrSpdPos->MSpd_s16 = rtrSpdPos->MSpd_RPM * RPM_TO_S16M; 
  if (rtrSpdPos->MSpd_s16 != 0){
	  rtrSpdPos->MPos_dpp = (uint16_t)(rtrSpdPos->MSpd_s16 * S16M_TO_MDPP);
	  rtrSpdPos->EPos_dpp = (uint16_t)(rtrSpdPos->MSpd_s16 * S16M_TO_EDPP);
	  rtrSpdPos->totalsectorCycles = (uint16_t)(SIXTY_DEG_Q16ANGLE/rtrSpdPos->EPos_dpp);
  }else{
	  rtrSpdPos->MPos_dpp = 0;
	  rtrSpdPos->EPos_dpp = 0;
	  rtrSpdPos->totalsectorCycles = 0;
  }
}

//This function has to be called in the HTF.The EPos_DPP function is calculated for that Freq.
void update_RtrPos(RotorSpeedPos *rtrSpdPos){
  rtrSpdPos->MAngle += rtrSpdPos->MPos_dpp; // 0-65535 => 0-359.99
  rtrSpdPos->EAngle += rtrSpdPos->EPos_dpp; // 0-65535 => 0-359.99
  //will roll over automatically.
  rtrSpdPos->sector = rtrSpdPos->EAngle/SIXTY_DEG_Q16ANGLE; //  goes from 0-5
  rtrSpdPos->sectorAngle = rtrSpdPos->EAngle - (rtrSpdPos->sector*SIXTY_DEG_Q16ANGLE);
  rtrSpdPos->currentCycle = rtrSpdPos->sectorAngle/rtrSpdPos->EPos_dpp;
}
  
void calculateSVPWM(RotorSpeedPos *rtrSpdPos, SVPWM *svpwm){
  //assume we have already calculated the sector and the sector angle outside.
  q15_t sinTheta = q15_sin(rtrSpdPos->sectorAngle); // sin(Theta)

  q15_t sinSixtyMinusTheta = q15_sin(SIXTY_DEG_Q16ANGLE - rtrSpdPos->sectorAngle);  //sin(60-theta)
  
  q15_t tempPV1 = q15_mul(sinSixtyMinusTheta,svpwm->modulationIndex);
  svpwm->PV1 = (uint16_t)(q15_to_float(tempPV1) * svpwm->CFG_timerCntMax);
    
  q15_t tempPV2 = q15_mul(sinTheta,svpwm->modulationIndex);
  svpwm->PV2 = (uint16_t)(q15_to_float(tempPV2) * svpwm->CFG_timerCntMax);
  
  svpwm->null = svpwm->CFG_timerCntMax - svpwm->PV1 - svpwm->PV2;
  //upto here takes 7-10uS
  
  svpwm->CNTA = svpwm->null/2;
  if (rtrSpdPos->sector%2 == 0){  // 1,3,5
      svpwm->CNTB = svpwm->CFG_timerCntMax - svpwm->CNTA - svpwm->PV2;
  }
  else{        				   // 0,2,4
      svpwm->CNTB = svpwm->CFG_timerCntMax - svpwm->CNTA - svpwm->PV1;
  }
  svpwm->CNTC = svpwm->CFG_timerCntMax - svpwm->CNTA;

}

void assign_SVPWM(RotorSpeedPos *rtrSpdPos, SVPWM *svpwm){
	/*
	Sector	Who fires first
	0		U	V	W
	1		V	U	W
	2		V	W	U
	3		W	V	U
	4		W	U	V
	5		U	W	V

	TODO CNTA/CNTB/CNTC have to be renamed to indicate what they are exactly -
	theyre the values of the first ,second and third intervals on the three phase PWM
	They dont have anything to do with U,V,W or phase 1/2/3
	*/

	if (rtrSpdPos->sector == 0){
		//U	 V	W
		svpwm->PhaseU = svpwm->CNTA;
		svpwm->PhaseV = svpwm->CNTB;
		svpwm->PhaseW = svpwm->CNTC;
	}
	else if(rtrSpdPos->sector == 1){
		//V	 U	W
		svpwm->PhaseU = svpwm->CNTB;
		svpwm->PhaseV = svpwm->CNTA;
		svpwm->PhaseW = svpwm->CNTC;
	}
	else if(rtrSpdPos->sector == 2){
		// V W	U
		svpwm->PhaseU = svpwm->CNTC;
		svpwm->PhaseV = svpwm->CNTA;
		svpwm->PhaseW = svpwm->CNTB;
	}
	else if(rtrSpdPos->sector == 3){
		// W V	U
		svpwm->PhaseU = svpwm->CNTC;
		svpwm->PhaseV = svpwm->CNTB;
		svpwm->PhaseW = svpwm->CNTA;
	}
	else if(rtrSpdPos->sector == 4){
		// W U	V
		svpwm->PhaseU = svpwm->CNTB;
		svpwm->PhaseV = svpwm->CNTC;
		svpwm->PhaseW = svpwm->CNTA;
	}
	else if(rtrSpdPos->sector == 5){
		// U W	V
		svpwm->PhaseU = svpwm->CNTA;
		svpwm->PhaseV = svpwm->CNTC;
		svpwm->PhaseW = svpwm->CNTB;
	}
	else{
		//Do Nothing
	}

}



  
