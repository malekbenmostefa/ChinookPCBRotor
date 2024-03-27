/**
 ******************************************************************************
 * File Name          : pitch.h
 * Description        : Module pour établir le lien de communication avec
 * 						l'encodeur de pitch A2K via le protocole proporietaire
 * 						SEI.
 * Created            : Mar 8, 2024
 * Author             : Malek Benmostefa et Cedric Verdi
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __PITCH_H__
#define __PITCH_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define ADDR_ENCODEUR 					0x0F	// Adresse de l'encodeur

//SEI Request Commands
#define REQUEST_POSITION 				0x01	// POSITION
#define REQUEST_POSITION_STATUS 		0x02	// POSITION + 1 byte STATUS
#define REQUEST_POSITION_TIME_STATUS 	0x03	// POSITION + 2 bytes TIMES + STATUS
#define REQUEST_STROBE 					0x04	// STROBE
#define REQUEST_SLEEP 					0x05	// SLEEP
#define REQUEST_WAKEUP 					0x06	// WAKEUP
#define REQUEST_MB_CMD 					0x0F	// MULTI BYTE COMMAND

//SEI Status
#define STATUS_OK						0x00
#define STATUS_NE_LIGHT					0x01	// NOT ENOUGH LIGHT
#define STATUS_TM_LIGHT					0x02	// TOO MUCH LIGHT
#define STATUS_MA_D_1					0x03	// MISALIGNMENT or DUST
#define STATUS_MA_D_2					0x04	// MISALIGNMENT or DUST
#define STATUS_MA_D_3					0x05	// MISALIGNMENT or DUST
#define STATUS_HW						0x06	// HARDWARE PROBLEM
#define STATUS_FM						0x07	// FAST MODE ERROR (V1.X)
#define STATUS_MULTITURN_INIT			0x08	// MUULTITURN POSITION NOT INITIALIZED


/* Type definitions ----------------------------------------------------------*/

/* Function prototypes ------------------------------------------------------ */

/**
 * @brief  Initialisation de l'encodeur
 * @param  uint8_t addr
 * @retval uint8_t
 */
uint8_t encodeur_Init(UART_HandleTypeDef *uart, uint8_t addr);


/**
 * @brief  Genere une commande de 8 bits avec le format «single byte» de SEI
 * @param  uint8_t addr (encoder address)
 * @param  uint8_t cmd (use SEI Request Commands)
 * @retval uint8_t
 */
uint8_t SB_cmd(uint8_t addr,uint8_t cmd);

/**
 * @brief  Envoie une commande de 8 bits avec le USART
 * @param  uint8_t cmd
 * @retval uint8_t (1 = send, 0 = not send)
 */
uint8_t transmit_SB_cmd(UART_HandleTypeDef *huart, uint8_t *cmd);

#endif
