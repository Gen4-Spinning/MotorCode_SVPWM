/*
 * Constants.h
 *
 *  Created on: 23-Feb-2023
 *      Author: harsha
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#ifndef Constants_H
#define Constants_H

#define MAXRPM 2000
#define RPM_TO_S16M 32
#define S16M_TO_RPM 0.03125


#define HTF_FOR_DPP 10000 // for CAPWM we can either not update at centre, or update at centre. 10000 is for not updating at centre
#define S16M_TO_EDPP 0.0171
#define S16M_TO_MDPP 0.0034

#define CNTS_TO_RPM_CONSTANT 1.875
#define CNTS_TO_S16_CONSTANT 60
#define CNTS_TO_MANGLE_S16 4

#define MANGLE_TO_EANGLE_MOD_DIVIDEND 13107
#define MANGLE_TO_EANGLE_MULTIPLIER 5

#endif

#endif /* INC_CONSTANTS_H_ */
