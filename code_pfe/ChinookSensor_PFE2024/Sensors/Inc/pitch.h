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
#define pitch_busy_line_Pin GPIO_PIN_4
#define pitch_busy_line_GPIO_Port GPIOB

#define ADDR_ENCODEUR 					0xF		// Adresse de l'encodeur (parle a tout les encodeurs de la ligne SEI)
#define RECEIVE_TIMEOUT					5		// Timeout pour laisser le temps à la premier donnée d'arrivé sur le USART RX (msec)
#define READ_CHECKSUM					0		// 1=Lire les checksum recu 0=Ne pas lire les checksum

//SEI Request Commands
#define REQUEST_POSITION 				0x1		// POSITION
#define REQUEST_POSITION_STATUS 		0x2		// POSITION + 1 byte STATUS
#define REQUEST_POSITION_TIME_STATUS 	0x3		// POSITION + 2 bytes TIMES + STATUS
#define REQUEST_STROBE 					0x4		// STROBE
#define REQUEST_SLEEP 					0x5		// SLEEP
#define REQUEST_WAKEUP 					0x6		// WAKEUP
#define REQUEST_MB_CMD 					0xF		// MULTI BYTE COMMAND

//SEI Status
#define STATUS_OK						0x0
#define STATUS_NE_LIGHT					0x1		// NOT ENOUGH LIGHT
#define STATUS_TM_LIGHT					0x2		// TOO MUCH LIGHT
#define STATUS_MA_D_1					0x3		// MISALIGNMENT or DUST
#define STATUS_MA_D_2					0x4		// MISALIGNMENT or DUST
#define STATUS_MA_D_3					0x5		// MISALIGNMENT or DUST
#define STATUS_HW						0x6		// HARDWARE PROBLEM
#define STATUS_FM						0x7		// FAST MODE ERROR (V1.X)
#define STATUS_MULTITURN_INIT			0x8		// MUULTITURN POSITION NOT INITIALIZED

//SEI Multiple Bytes Commands
#define MODE_SET_ORIGIN					0x01	// Set Origin Command
#define MODE_SET_ABS_POSITION			0x02	// Set Absolute Position Command
#define MODE_READ_SN					0x03	// Read Serial Number
#define MODE_CHECK_SN					0x04	// Check Serial Number
#define MODE_FAIL_SN					0x05	// Fail Serial Number
#define MODE_GET_ADDRESS				0x06	// Get Address
#define MODE_SET_ADDRESS				0x07	// Assign Address
#define MODE_READ_FI					0x08	// Read Factory Info
#define MODE_READ_RESOLUTION			0x09	// Read Resolution Command
#define MODE_SET_RESOLUTION				0x0A	// Change Resolution Command
#define MODE_READ_MODE					0x0B	// Read Mode Command
#define MODE_SET_MODE					0x0C	// Change Mode Command
#define MODE_CHANGE_POWER_UP			0x0D	// Change Power Up Mode Command
#define MODE_RESET						0x0E	// Reset Command
#define MODE_LOOPBACK					0x10	// Loopback Mode
#define MODE_OFF_LINE					0x11	// Off line Command
#define MODE_BAUD_RATE					0x0F	// Change Baud Rate Command (temporary)

//SEI Mode
#define MODE_REV						0x0		// 1 = Position increase counter clockwise, 0 = Position increase clockwise
#define MODE_STB						0x1		// 1 = Strobe mode on, 0 = Strobe mode off
#define MODE_MULTI						0x2		// 1 = multi-turnmode, 0 = single turn mode
#define MODE_SIZE						0x3		// 1 = send position in 2 bytes always, 0 = 1 bytes id position less than 256 or 2 bytes if position more than 256
#define MODE_INCR						0x4		// only in multi-turn mode, 1 = sends position change since last request, 0 = send position

//Length received bytes
#define POSITION_LENGTH					2		// Position length is fix on 2 bytes during init phase
#define STATUT_LENGTH					1		// Status length is 1 byte
#define TIME_LENGTH						2 		// Time length is 2 bytes

//Baud Rate Commands
#define BR_1200							0x15	// Baud Rate at 1200
#define BR_2400							0x14	// Baud Rate at 2400
#define BR_4800							0x13	// Baud Rate at 4800
#define BR_9600							0x12	// Baud Rate at 9600
#define BR_19200						0x11	// Baud Rate at 19200
#define BR_38400						0x10	// Baud Rate at 38400
#define BR_57600						0x01	// Baud Rate at 57600
#define BR_115200						0x00	// Baud Rate at 115200

//Baud Rate Commands
#define SINGLE_TURN_MODE				0
#define MULTI_TURN_MODE					1

/* Type definitions ----------------------------------------------------------*/


/* Function prototypes ------------------------------------------------------ */

/**
 * @brief  Calcul la valeur du checksum attendue
 * @param  uint8_t data
 * @retval uint8_t
 */
uint8_t calculate_checksum(uint8_t data);

/**
 * @brief  Genere une commande de 8 bits avec le format «single byte» de SEI
 * @param  uint8_t addr (encoder address)
 * @param  uint8_t cmd (use SEI Request Commands)
 * @retval uint8_t
 */
uint8_t SB_cmd(uint8_t addr,uint8_t cmd);

/**
 * @brief  Envoie une commande de 8 bits avec le USART
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t* request
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_send_SB_cmd(UART_HandleTypeDef *huart, uint8_t request);

/**
 * @brief  Sets the absolute 0 at the current position
 * 			In single-turn mode, the new origin is stored in EEPROM, therefore, it will be effective after resets
 *			and power downs until a “Set Origin” or a “Set Absolute Position” command is received.
 *			In multi-turn mode, the 32 bit counter is reset, but not stored in EEPROM. This is effective until a
 *			reset occurs or a “Set Origin” or a “Set Absolute Position” command is received.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_set_origin(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  Sets the given absolute position (at the current resolution) at the current position.
 *			In single-turn mode, the new origin is stored in EEPROM, therefore it will be effective after resets
 *			and power downs, until a “Set Origin” or a “Set Absolute Position” command is received.
 *			In multi-turn mode, the 32 bit counter is set, but not stored in EEPROM. This is effective until a
 *			reset occurs or a “Set Origin” or a “Set Absolute Position” command is received.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t turn_mode
 * @param  uint8_t *position
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_set_absolute_position(UART_HandleTypeDef *huart, uint8_t addr, uint8_t turn_mode, uint8_t *position);

/**
 * @brief  Read the Serial Number of a specific encoder
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_read_serial_number(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  The encoder does a logical AND of its serial number with the mask supplied; the result is
 *		   compared to the serial number supplied. If they match, the busy line is held active until another
 *		   byte is received. Otherwise the busy line is released. This command is used to determine if an
 *         encoder with a particular serial number is present on the bus.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t *serial_number
 * @param  uint8_t *mask
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_check_serial_number(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t *mask);

/**
 * @brief  The encoder does a logical AND of its serial number with the mask supplied; the result is
 *		   compared to the serial number supplied. If they dont match, the busy line is held active until
 *		   another byte is received. If they match the busy line is released. This is useful to determine if an
 *		   encoder, whose serial number is known, is the only one on the bus.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t *serial_number
 * @param  uint8_t *mask
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_fail_serial_number(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t *mask);

/**
 * @brief  The encoder compares its serial number with the one supplied; if they match, it returns its address
 *		   (0 to E). Otherwise, it returns nothing.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t *serial_number
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_get_address(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number);

/**
 * @brief  The encoder compares its serial number with the one supplied; if they match, it assigns itself the
 *		   address supplied (must be between 0 and E). The new address is stored in EEPROM, therefore, it
 * 		   will be effective after resets and power downs.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t *serial_number
 * @param  uint8_t new_addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_assign_address(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *serial_number, uint8_t new_addr);

/**
 * @brief  Read the factory info : model number, version, configuration, serial
 *		   number, month, day, year
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_read_factory_info(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  Resolution MS byte, resolution LS byte and checksum if command is successful.
 *		   A zero value means 16 bit resolution.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_read_resolution(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  The resolution can be any number between 0 and FFFF, 0 is for full 16 bit position. However, the
 *		   accuracy is only guaranteed to 12 bits. The new resolution is stored in EEPROM, therefore, it will be
 *		   effective after resets and power downs.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t *resolution
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_change_resolution(UART_HandleTypeDef *huart, uint8_t addr, uint8_t *resolution);

/**
 * @brief  Read the present mode settings.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_read_mode(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  The mode is changed temporarily and will be effective until the encoder is reset, power down, or
 *		   another mode change command is received. It is not stored in the EEPROM.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t mode
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_change_mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t mode);

/**
 * @brief  Same as “Change Mode Command” described above, except the mode is stored in EEPROM,
 *		   therefore it will be effective across resets and power cycles.
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t mode
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_change_power_up_mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t mode);

/**
 * @brief  After releasing the busy line the encoder does a software reset (the baud rate returns to 9600
 *		   after the checksum byte is sent).
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_reset(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  After releasing the busy line the encoder does a software reset (the baud rate returns to 9600
 *		   after the checksum byte is sent).
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_loopback_mode(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  After releasing the busy line the encoder does a software reset (the baud rate returns to 9600
 *		   after the checksum byte is sent).
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_off_line(UART_HandleTypeDef *huart, uint8_t addr);

/**
 * @brief  After releasing the busy line the encoder does a software reset (the baud rate returns to 9600
 *		   after the checksum byte is sent).
 * @param  UART_HandleTypeDef *huart
 * @param  uint8_t addr
 * @param  uint8_t baud_rate
 * @retval uint8_t (1 = no error, 0 = error)
 */
uint8_t pitch_change_baud_rate(UART_HandleTypeDef *huart, uint8_t addr, uint8_t baud_rate);

/**
 * @brief  Initialisation de l'encodeur
 * @param  UART_HandleTypeDef *uart
 * @param  uint8_t addr
 * @retval uint8_t
 */
void pitch_Init(UART_HandleTypeDef *uart, uint8_t addr);

#endif
