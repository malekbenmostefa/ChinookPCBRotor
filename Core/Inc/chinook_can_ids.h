/*
 * chinook_can_ids.h
 *
 *  Created on: May 24, 2022
 *      Author: Marc
 */

#ifndef INC_CHINOOK_CAN_IDS_H_
#define INC_CHINOOK_CAN_IDS_H_


//
// MARIO Tx CAN IDs
//

// Sensor data
#define MARIO_PITCH_ANGLE 0x01
#define MARIO_MAST_ANGLE 0x02
#define MARIO_TURBINE_RPM 0x03
#define MARIO_WHEEL_RPM 0x04
#define MARIO_WIND_DIRECTION 0x05
#define MARIO_WIND_SPEED 0x06

#define MARIO_LOADCELL 0x09
#define MARIO_TORQUE 0x0A
#define MARIO_LIMIT_SWITCH 0x0B

// Motor modes
#define MARIO_PITCH_MODE_CMD 0x11
#define MARIO_MAST_MODE_CMD 0x12
#define MARIO_ROPS_CMD 0x13

// Motor Control
#define MARIO_PITCH_CMD 0x17
#define MARIO_MAST_CMD 0x18
#define MARIO_PITCH_EMERGENCY_STOP 0x19
#define MARIO_MAST_EMERGENCY_STOP 0x1A
#define MARIO_DRIVE_MOTOR_RESET 0x1B

#define MARIO_RX_FILTER_ID_HIGH 0x0000
#define MARIO_RX_FILTER_ID_LOW  0x0000
#define MARIO_RX_FILTER_MASK_HIGH 0xFFFF
#define MARIO_RX_FILTER_MASK_LOW  0xFFE0

//
// Volant Tx CAN IDs
//

// Motor modes
#define VOLANT_PITCH_MODE_CMD 0x31
#define VOLANT_MAST_MODE_CMD 0x32

// Motor Manual Control commands
#define VOLANT_MANUAL_PITCH_DIR 0x33
#define VOLANT_MANUAL_MAST_DIR 0x34
#define VOLANT_MANUAL_ROPS_CMD 0x35

#define VOLANT_RX_FILTER_ID_HIGH 0x0000
#define VOLANT_RX_FILTER_ID_LOW  0x0030
#define VOLANT_RX_FILTER_MASK_HIGH 0xFFFF
#define VOLANT_RX_FILTER_MASK_LOW  0xFFF0

//
// Backplane CAN IDs
//

// Board sensors
#define BACKPLANE_TOTAL_VOLTAGE 0x51
#define BACKPLANE_TOTAL_CURRENT 0x52
#define BACKPLANE_BOARD_VOLTAGES_1 0x53
#define BACKPLANE_BOARD_VOLTAGES_2 0x54
#define BACKPLANE_BOARD_CURRENTS_1 0x55
#define BACKPLANE_BOARD_CURRENTS_2 0x56
#define BACKPLANE_VOLANT_VOLTAGE 0x57
#define BACKPLANE_VOLANT_CURRENT 0x58
#define BACKPLANE_BOARD_SLOTS_HS 0x59

#define BACKPLANE_VOLANT_STATUS 0x5A

// Backplane Buzzer Cmd
#define BACKPLANE_BUZZER_CMD 0x5B

#define BACKPLANE_RX_FILTER_ID_HIGH 0x0000
#define BACKPLANE_RX_FILTER_ID_LOW  0x0050
#define BACKPLANE_RX_FILTER_MASK_HIGH 0xFFFF
#define BACKPLANE_RX_FILTER_MASK_LOW  0xFFF0


//
// Drive Moteur Tx CAN IDs
//

// Motor modes feedback
#define DRIVEMOTOR_PITCH_MODE_FEEDBACK 0x71
#define DRIVEMOTOR_MAST_MODE_FEEDBACK 0x72

// BEMF sensors
#define DRIVEMOTOR_PITCH_BEMF 0x73
#define DRIVEMOTOR_MAST_BEMF 0x74

// Motor control feedback
#define DRIVEMOTOR_PITCH_DONE 0x75

#define DRIVEMOTOR_RX_FILTER_ID_HIGH 0x0000
#define DRIVEMOTOR_RX_FILTER_ID_LOW  0x0070
#define DRIVEMOTOR_RX_FILTER_MASK_HIGH 0xFFFF
#define DRIVEMOTOR_RX_FILTER_MASK_LOW  0xFFF0



#endif /* INC_CHINOOK_CAN_IDS_H_ */
