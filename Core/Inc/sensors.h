/*
 * sensors.h
 *
 *  Created on: 11 juin 2023
 *      Author: Marc Beaulieu
 */

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "stm32f4xx_hal.h"

// Need to export because interrupts initialization needs address for writing read bytes
extern uint8_t ws_rx_byte[4];


void ReadWeatherStation();

// float ReadTorqueADC();
// float ReadLoadcellADC();
void ReadTorqueLoadcellADC();

float GetWheelRPM();
float CalcVehicleSpeed(float wheel_rpm);

float GetRotorRPM();

uint32_t ReadPitchEncoder();
uint32_t ReadMastEncoder();



#endif /* _SENSORS_H_ */
