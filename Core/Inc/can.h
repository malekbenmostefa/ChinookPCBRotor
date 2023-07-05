/*
 * can.h
 *
 *  Created on: 15 juin 2023
 *      Author: Marc
 */

#ifndef _CAN_H_
#define _CAN_H_

#include "main.h"
#include "chinook_can_ids.h"

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority);


#endif /* _CAN_H_ */
