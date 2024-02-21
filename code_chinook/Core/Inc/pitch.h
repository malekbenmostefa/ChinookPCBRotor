/*
 * pitch.h
 *
 *  Created on: 13 juin 2023
 *      Author: Marc
 */

#ifndef _PITCH_H_
#define _PITCH_H_

#include "stm32f4xx_hal.h"


#define MAX_PITCH_VALUE 4194303
//#define MAX_PITCH_VALUE 4095
#define HALF_MAX_VALUE 2097152
//#define HALF_MAX_VALUE 2047

//#define PITCH_ABSOLUTE_ZERO 1668850
//#define PITCH_ABSOLUTE_ROPS 2520743
#define PITCH_ABSOLUTE_ZERO (580*1024)
// #define PITCH_ABSOLUTE_ROPS (2500*1024)
#define PITCH_ABSOLUTE_ROPS (1260*1024)

#define MAX_STEPS_PER_CMD 500


float CalcPitchAnglePales(uint8_t bound_angle);
float CalcPitchAnglePales(uint32_t abs_target, uint8_t bound_angle);

float CalcTSR();
float CalcPitchAuto();

void VerifyPitchTargetCmd(uint32_t target_pitch_abs);

void VerifyRopsCmd();
void SendPitchTargetCmd(uint32_t target_pitch_abs);
void SendPitchROPSCmd();
void SendPitchAngleCmd(float target_pitch);


#endif /* _PITCH_H_ */
