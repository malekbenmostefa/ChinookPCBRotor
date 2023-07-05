/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <math.h>

#include "sensors.h"
#include "pitch.h"
#include "can.h"
#include "motor_control.h"

#include "chinook_can_ids.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ROTOR_RPM_ROPS 800

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

enum STATES
{
	STATE_INIT = 0,
	STATE_ACQUISITION,
	STATE_CHECK_ROPS,
	STATE_MOTOR_CONTROL,
	STATE_CAN,
	STATE_DATA_LOGGING,
	STATE_UART_TX,

	STATE_ROPS,

	STATE_ERROR = 0xFF

};
uint32_t current_state = STATE_INIT;

SensorData sensor_data;


uint8_t timer_50ms_flag = 0;
uint8_t timer_100ms_flag = 0;
uint8_t timer_500ms_flag = 0;

uint8_t flag_lora_tx_send = 0;
uint8_t flag_can_tx_send = 0;
uint8_t flag_acq_interval = 0;
uint8_t flag_rotor_rpm_process = 0;
uint8_t flag_wheel_rpm_process = 0;
uint8_t flag_alive_led = 0;
uint8_t flag_uart_tx_send = 0;

float pitch_auto_target;
float pitch_rops_target;
uint8_t pitch_done = 0;
uint8_t b_rops = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

void ExecuteStateMachine();

uint32_t DoStateInit();
uint32_t DoStateAcquisition();
uint32_t DoStateMotorControl();
uint32_t DoStateROPS();
uint32_t DoStateCAN();
uint32_t DoStateDataLogging();
uint32_t DoStateUartTx();

void DoStateError();

void FloatToString(float value, int decimal_precision, unsigned char* val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t delay16_us)
{
	htim1.Instance->CNT = 0;
	while (htim1.Instance->CNT < delay16_us);
}

void delay_ms(uint16_t delay16_ms)
{
	while(delay16_ms > 0)
	{
		htim1.Instance->CNT = 0;
		delay16_ms--;
		while (htim1.Instance->CNT < 1000);
	}
}

void ExecuteStateMachine()
{
	static int timer_250ms_counter = 0;
	if (timer_50ms_flag)
	{
		timer_250ms_counter++;
		timer_50ms_flag = 0;
	}
	if (timer_250ms_counter == 5)
	{
		flag_uart_tx_send = 1;
		timer_250ms_counter = 0;
	}
	if (timer_100ms_flag)
	{
		timer_100ms_flag = 0;

		flag_can_tx_send = 1;
		flag_acq_interval = 1;
		flag_rotor_rpm_process = 1;
		// flag_uart_tx_send = 1;
	}
	if (timer_500ms_flag)
	{
		timer_500ms_flag = 0;

		flag_wheel_rpm_process = 1;
		flag_alive_led = 1;
	}

	// Alive led
	if (flag_alive_led)
	{
		flag_alive_led = 0;
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}

	switch (current_state)
	{
	case STATE_INIT:
		current_state = DoStateInit();
		break;

	case STATE_ACQUISITION:
		current_state = DoStateAcquisition();
		break;

	case STATE_MOTOR_CONTROL:
		current_state = DoStateMotorControl();
		break;

	case STATE_ROPS:
		current_state = DoStateROPS();
		break;

	case STATE_CAN:
		current_state = DoStateCAN();
		break;

	case STATE_DATA_LOGGING:
		current_state = DoStateDataLogging();
		break;

	case STATE_UART_TX:
		current_state = DoStateUartTx();
		break;

	case STATE_ERROR:
		DoStateError();
		current_state = DoStateInit();
		break;

	default:
		current_state = DoStateInit();
		break;
	}
}

uint32_t DoStateInit()
{
	// Reset all sensor data
	memset(&sensor_data, 0, sizeof(SensorData));

	// Causes strange bug where stm32 is still executing interrupts but not main code ....
	// Start interrupts
	HAL_UART_Receive_IT(&huart5, &ws_rx_byte[0], 1);

	// TODO: (Marc) Not sure why TIM1 is started in its Init function and not here. Need to make consistent
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);

	// Enable USB TX
	HAL_GPIO_WritePin(USB_TX_EN_GPIO_Port, USB_TX_EN_Pin, GPIO_PIN_SET);

	// Reset USB line
	HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);


	return STATE_ACQUISITION;
}

uint32_t DoStateAcquisition()
{
	if (ws_receive_flag)
	{
		ReadWeatherStation();
	}

	if (flag_acq_interval)
	{
		flag_acq_interval = 0;

		// Read pitch and mast encoders
		uint32_t pitch_read = ReadPitchEncoder();
		if (pitch_read != 0xFFFFFFFF)
			sensor_data.pitch_encoder = pitch_read;
		sensor_data.mast_encoder = ReadMastEncoder();
		//TODO: (Marc) Mast Encoder
		sensor_data.mast_encoder = 0;

		sensor_data.pitch_angle = CalcPitchAnglePales(TRUE);


		// Read limit swtiches of mast
		sensor_data.limit1 = HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin);
		sensor_data.limit2 = HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin);

		ReadTorqueLoadcellADC();

		// Power
		static const float RPM_TO_RADS = 2.0f * PI / 60.0f;
		float rotor_speed_rads = RPM_TO_RADS * sensor_data.rotor_rpm;
		sensor_data.power = sensor_data.torque * rotor_speed_rads;
		if (sensor_data.power < MIN_EPSILON)
			sensor_data.power = 0.0f;
	}

	if (flag_wheel_rpm_process)
	{
		flag_wheel_rpm_process = 0;

		sensor_data.wheel_rpm = GetWheelRPM();
		sensor_data.vehicle_speed = CalcVehicleSpeed(sensor_data.wheel_rpm);
	}

	if (flag_rotor_rpm_process)
	{
		flag_rotor_rpm_process = 0;

		sensor_data.rotor_rpm = GetRotorRPM();
	}

	if (b_rops)
		return STATE_ROPS;
	else
		return STATE_MOTOR_CONTROL;

	// Check ROPS
	if (sensor_data.rotor_rpm >= ROTOR_RPM_ROPS)
	{
		// ACTIVATE ROPS
		b_rops = 1;
		return STATE_ROPS;
	}

	return STATE_MOTOR_CONTROL;
}

uint32_t DoStateMotorControl()
{
#define NORMAL 0
#define ALL_MANUAL 0
#define ALL_AUTO 0
#define MANUAL_MAST 0
#define TEST_PITCH_MANUAL 0
#define TEST_PITCH_AUTO 0
#define TEST_AUTO_ROPS 1

	if (NORMAL)
	{
		// TODO: Code compe
		if (!b_rops)
		{
			DoPitchControl();
			// VerifyPitchTargetCmd(PITCH_ABSOLUTE_ZERO);
			DoMastControl();
		}
		else
		{
			VerifyRopsCmd();
		}
	}
	else if (ALL_MANUAL)
	{
		if (sensor_data.feedback_pitch_mode != MOTOR_MODE_MANUAL)
		{
			uint32_t pitch_mode = MOTOR_MODE_MANUAL;
			TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
			delay_us(100);
		}
		if (sensor_data.feedback_mast_mode != MOTOR_MODE_MANUAL)
		{
			uint32_t mast_mode = MOTOR_MODE_MANUAL;
			TransmitCAN(MARIO_MAST_MODE_CMD, (uint8_t*)&mast_mode, 4, 0);
			delay_us(100);
		}
	}
	else if (ALL_AUTO)
	{
		if (sensor_data.feedback_mast_mode != MOTOR_MODE_AUTOMATIC)
		{
			uint32_t mast_mode = MOTOR_MODE_AUTOMATIC;
			TransmitCAN(MARIO_MAST_MODE_CMD, (uint8_t*)&mast_mode, 4, 0);
			delay_us(100);
		}
		if (sensor_data.feedback_pitch_mode != MOTOR_MODE_AUTOMATIC)
		{
			uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
			TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
			delay_us(100);
		}
	}
	else if (MANUAL_MAST)
	{
		uint32_t dir_left = MOTOR_DIRECTION_LEFT;
		uint32_t dir_right = MOTOR_DIRECTION_RIGHT;
		uint32_t dir_stop = MOTOR_DIRECTION_STOP;

		// static uint32_t manual_motor_id = MARIO_PITCH_MANUAL_CMD;
		static uint32_t manual_motor_id = MARIO_MAST_MANUAL_CMD;

		static int left_on = 0;
		static int right_on = 0;

		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) && right_on) // PD_14 -- PB2
		{
			//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
			//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			//if (pb2_value == 1)
			{
				right_on = 0;
				pb2_value = 0;
				// HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

				TransmitCAN(manual_motor_id, (uint8_t*)&dir_stop, 4, 0);
			}
		}
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) && left_on) // PD_15 -- PB1
		{
			//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			// if (pb1_value == 1)
			{
				left_on = 0;
				pb1_value = 0;
				// HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

				TransmitCAN(manual_motor_id, (uint8_t*)&dir_stop, 4, 0);
			}
		}

		if (pb1_update)
		{
			pb1_update = 0;
			TransmitCAN(manual_motor_id, (uint8_t*)&dir_left, 4, 0);
			left_on = 1;
		}
		else if (pb2_update)
		{
			pb2_update = 0;
			TransmitCAN(manual_motor_id, (uint8_t*)&dir_right, 4, 0);
			right_on = 1;
		}
		/*
		else if (left_on || right_on)
		{
			left_on = right_on = 0;
			TransmitCAN(manual_motor_id, (uint8_t*)&dir_stop, 4, 1);
		}
		*/
	}
	else if (TEST_PITCH_MANUAL)
	{
		if (!b_rops)
		{
			if (sensor_data.feedback_pitch_mode != MOTOR_MODE_MANUAL)
			{
				uint32_t pitch_mode = MOTOR_MODE_MANUAL;
				TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
				delay_us(100);
			}
		}
	}
	else if (TEST_PITCH_AUTO)
	{
		if (!b_rops)
		{
			// Pitch auto control
			static uint8_t target_changed = 0;
			if (pb1_update || pb2_update)
			{
				target_changed = 1;

				if (sensor_data.feedback_pitch_mode != MOTOR_MODE_AUTOMATIC)
				{
					uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
					TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
					delay_us(100);
				}
			}

			static float target_pitch = 0.0f;

			float target_pitch0 = 0.0f;
			float target_pitch1 = 10.0f;

			if (pb1_update)
			{
				pb1_update = 0;
				target_pitch = target_pitch0;
				// SendPitchAngleCmd(target_pitch0);
			}
			if (pb2_update)
			{
				pb2_update = 0;
				target_pitch = target_pitch1;
				// SendPitchAngleCmd(target_pitch1);
			}

			if (target_changed)
			{
				// Compute delta angle from -180 to 180 degrees
				float delta_angle_pales = CalcPitchAnglePales(TRUE) - target_pitch;
				// delta_angle_pales = BoundAngleSemiCircle(delta_angle_pales);

	#define MIN_ERROR_ANGLE 0.1f
				if (abs(delta_angle_pales) > 0.1f)
				{
					if (pitch_done)
					{
						// Small delay inbetween commands for smoothness
						static uint32_t pitch_done_counter = 0;

						++pitch_done_counter;

	#define PITCH_DONE_WAIT_NUM 20
						if (pitch_done_counter >= PITCH_DONE_WAIT_NUM)
						{
							pitch_done_counter = 0;
							SendPitchAngleCmd(target_pitch);
						}
					}
				}
			}
		}
	}
	else if (TEST_AUTO_ROPS)
	{
		// if (!b_rops)
		{
			// Pitch auto control
			static uint8_t target_changed = 0;
			if (pb1_update || pb2_update)
			{
				target_changed = 1;

				if (sensor_data.feedback_pitch_mode != MOTOR_MODE_AUTOMATIC)
				{
					uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
					TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
					delay_us(100);
				}
			}

			static float target_pitch = 0.0f;
			static uint8_t target_drapeau = 0;

			// float target_pitch0 = 0.0f;
			// float target_pitch1 = 10.0f;

			if (pb1_update)
			{
				pb1_update = 0;
				// target_pitch = target_pitch0;
				target_drapeau = 1;
				// b_drapeau = 1;
				// SendPitchAngleCmd(target_pitch0);
			}
			if (pb2_update)
			{
				pb2_update = 0;
				// target_pitch = target_pitch1;
				target_drapeau = 0;
				// b_drapeau = 0;
				// SendPitchAngleCmd(target_pitch1);
			}

			if (target_changed)
			{
				if (target_drapeau)
				{
					VerifyRopsCmd();
				}
				else
				{
// #define MAX_ERROR_ROPS 10000
					VerifyPitchTargetCmd(PITCH_ABSOLUTE_ZERO);

					/*
					float delta_angle_pales = CalcPitchAnglePales(TRUE) - 0.0f;

#define MIN_ERROR_ANGLE 0.1f
					if (abs(delta_angle_pales) > 0.1f)
					{
						if (pitch_done)
						{
							// Small delay inbetween commands for smoothness
							static uint32_t pitch_done_counter = 0;
							++pitch_done_counter;

#define PITCH_DONE_WAIT_NUM_ROPS 6
							if (pitch_done_counter >= PITCH_DONE_WAIT_NUM_ROPS)
							{
								pitch_done_counter = 0;
								SendPitchAngleCmd(target_pitch);
							}
						}

					}
					*/
				}
				/*
				// Compute delta angle from -180 to 180 degrees
				float delta_angle_pales = CalcPitchAnglePales(TRUE) - target_pitch;
				delta_angle_pales = BoundAngleSemiCircle(delta_angle_pales);

	#define MIN_ERROR_ANGLE 0.1f
				if (abs(delta_angle_pales) > 0.1f)
				{
					if (pitch_done)
					{
						// Small delay inbetween commands for smoothness
						static uint32_t pitch_done_counter = 0;

						++pitch_done_counter;

	#define PITCH_DONE_WAIT_NUM 20
						if (pitch_done_counter >= PITCH_DONE_WAIT_NUM)
						{
							pitch_done_counter = 0;
							SendPitchAngleCmd(target_pitch);
						}
					}
				}
				*/
			}
		}
	}

	return STATE_CAN;
}

uint32_t DoStateROPS()
{
	DoStateAcquisition();
	DoStateMotorControl();
	DoStateCAN();
	DoStateDataLogging();
	DoStateUartTx();

	pitch_rops_target = CalcPitchAnglePales(PITCH_ABSOLUTE_ROPS, TRUE);

	// Make sure we stay in pitch auto mode for ROPS
	if (sensor_data.feedback_pitch_mode != MOTOR_MODE_AUTOMATIC)
	{
		uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
		TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
		delay_us(100);
	}

	if (!b_rops)
	{
		pitch_rops_target = 0;
		// if (sensor_data.feedback_pitch_rops == 1)
		{
			uint32_t rops_cmd = ROPS_DISABLE;
			SendROPSCmdCan(rops_cmd);
		}

		return STATE_ACQUISITION;
	}

	return STATE_ROPS;
}

uint32_t DoStateCAN()
{
	/*
	sensor_data.pitch_encoder = 2900;
	sensor_data.wind_speed = 5.67f;
	sensor_data.torque = 12.34f;
	sensor_data.loadcell = 67.89f;
	sensor_data.rotor_rpm = 156.7f;
	*/
	if (flag_can_tx_send) // Sent every 100ms
	{
		flag_can_tx_send = 0;

		// uint32_t gear = 4;
		// TransmitCAN(MARIO_GEAR_TARGET, (uint8_t*)&gear, 4, 0);
		// delay_us(100);

		// float pitch_angle = 23.89f;
		// TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&pitch_angle, 4, 0);
		// sensor_data.pitch_angle = CalcPitchAnglePales(TRUE);
		// sensor_data.pitch_angle = 40.22f;


		float pitch_val = sensor_data.pitch_angle;
		// float pitch_val = ((float)sensor_data.pitch_encoder / 1000.0f) / 1024.0f;
		// pitch_val = 46.79f;
		TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&pitch_val, 4, 0);
		delay_us(100);

		// float rotor_rpm = 234.6f;
		// TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&rotor_rpm, 4, 0);
		// static float value_test = 0;
		TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
		// ++value_test;
		// TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
		delay_us(100);

		// sensor_data.torque = sensor_data.loadcell;
		// TransmitCAN(MARIO_TORQUE, (uint8_t*)&sensor_data.torque, 4, 0);
		// delay_us(100);

		// TransmitCAN(MARIO_LOADCELL, (uint8_t*)&sensor_data.loadcell, 4, 0);
		// delay_us(100);



		// Power = 328.44f
		// float torque = 0.0f;
		// TransmitCAN(MARIO_TORQUE, (uint8_t*)&torque, 4, 0);
		delay_us(100);

		// static float wind_speed = 12.78f;
		// wind_speed += 0.1f;
		// TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&wind_speed, 4, 0);

		float wind_speed_ms = sensor_data.wind_speed;
		TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&wind_speed_ms, 4, 0);
		delay_us(100);

		// float wheel_rpm_adj = sensor_data.wheel_rpm + 1;
		// TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&wheel_rpm_adj, 4, 0);
		// delay_us(100);


		float tsr = CalcTSR();
		// float tsr = 12.34f;

		// tsr = sensor_data.torque;
		// TransmitCAN(MARIO_TIP_SPEED_RATIO, (uint8_t*)&tsr, 4, 0);
		// delay_us(100);

		float wind_dir = sensor_data.wind_direction - 120.0f;
		// float wind_dir = sensor_data.loadcell;
		TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&wind_dir, 4, 0);
		delay_us(100);

		uint8_t rops_feedback = b_rops;
		TransmitCAN(MARIO_ROPS_FEEDBACK, (uint8_t*)&rops_feedback, 4, 0);
		delay_us(100);


		TransmitCAN(MARIO_PITCH_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_pitch_mode, 4, 0);
		delay_us(100);


		TransmitCAN(MARIO_MAST_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_mast_mode, 4, 0);
		delay_us(100);


		// TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&wind_dir, 4, 0);
		// delay_us(100);

		/*
		// Motor modes feedback to volant
// #define MARIO_PITCH_MODE_FEEDBACK 0x4C
// #define MARIO_MAST_MODE_FEEDBACK 0x4D
		TransmitCAN(MARIO_PITCH_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_pitch_mode, 4, 0);
		delay_us(100);

		TransmitCAN(MARIO_MAST_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_mast_mode, 4, 0);
		delay_us(100);
		*/

		// DEBUG DEBUG -- CAN Volant
		/*
		if (flag_can_tx_send) // Sent every 100ms
		{
			flag_can_tx_send = 0;

			static const uint8_t uint_buffer_test[] = { 111, 112, 113, 114, 115, 116, 117, 118, 119, 210 };
			static const float float_buffer_test[] = { 9.10f, 9.20f, 9.30f, 9.40f, 9.50f, 9.60f, 9.70f, 9.80f, 9.90f };

			uint8_t uint_buffer_index = 0;
			uint8_t float_buffer_index = 0;

			TransmitCAN(MARIO_GEAR_TARGET, (uint8_t*)&uint_buffer_test[uint_buffer_index++], 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&float_buffer_test[float_buffer_index++], 4, 0);
			delay_us(50);

			// TransmitCAN(MARIO_MAST_ANGLE, (uint8_t*)&sensor_data.mast_angle, 4, 0);
			// delay_us(50);

			// TransmitCAN(MARIO_MAST_ANGLE, (uint8_t*)&sensor_data.vehicle_speed, 4, 0);
			// delay_us(50);

			TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&float_buffer_test[float_buffer_index++], 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&float_buffer_test[float_buffer_index++], 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&float_buffer_test[float_buffer_index++], 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&float_buffer_test[float_buffer_index++], 4, 0);
			delay_us(50);

			// TransmitCAN(MARIO_TORQUE, (uint8_t*)&sensor_data.torque, 4, 0);
			// delay_us(50);

			// TransmitCAN(MARIO_LOADCELL, (uint8_t*)&sensor_data.loadcell, 4, 0);
			// delay_us(50);

			// TODO: (Marc) Batt voltage + Batt current
			// TODO: (Marc) Limit switch

			static const uint8_t mode_test_manual = MOTOR_MODE_MANUAL;
			static const uint8_t mode_test_automatic = MOTOR_MODE_AUTOMATIC;

			TransmitCAN(MARIO_PITCH_MODE_FEEDBACK, (uint8_t*)&mode_test_manual, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_MAST_MODE_FEEDBACK, (uint8_t*)&mode_test_automatic, 4, 0);
			delay_us(50);

			// static const uint8_t rops_test = ROPS_ENABLE;
			static const uint8_t rops_test = 1;

			TransmitCAN(MARIO_ROPS_FEEDBACK, (uint8_t*)&rops_test, 4, 0);
			delay_us(50);

			// Also send the turbine rpm value to the drive motor for ROPS detection
			// TransmitCAN(MARIO_MOTOR_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
			// delay_us(50);
		}
		*/


		/*
		if (flag_can_tx_send) // Sent every 100ms
		{
			flag_can_tx_send = 0;

			uint32_t gear_target = 0;
			TransmitCAN(MARIO_GEAR_TARGET, (uint8_t*)&gear_target, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&sensor_data.pitch_angle, 4, 0);
			delay_us(50);

			// TransmitCAN(MARIO_MAST_ANGLE, (uint8_t*)&sensor_data.mast_angle, 4, 0);
			// delay_us(50);

			// TransmitCAN(MARIO_MAST_ANGLE, (uint8_t*)&sensor_data.vehicle_speed, 4, 0);
			// delay_us(50);

			TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&sensor_data.wheel_rpm, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&sensor_data.wind_direction, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&sensor_data.wind_speed, 4, 0);
			delay_us(50);

			static float tsr = 24.1f;
			tsr += 0.1f;
			TransmitCAN(MARIO_TIP_SPEED_RATIO, (uint8_t*)&tsr, 4, 0);
			delay_us(50);

			// TransmitCAN(MARIO_TORQUE, (uint8_t*)&sensor_data.torque, 4, 0);
			// delay_us(50);

			// TransmitCAN(MARIO_LOADCELL, (uint8_t*)&sensor_data.loadcell, 4, 0);
			// delay_us(50);

			// TODO: (Marc) Batt voltage + Batt current
			// TODO: (Marc) Limit switch

			TransmitCAN(MARIO_PITCH_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_pitch_mode, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_MAST_MODE_FEEDBACK, (uint8_t*)&sensor_data.feedback_mast_mode, 4, 0);
			delay_us(50);

			TransmitCAN(MARIO_ROPS_FEEDBACK, (uint8_t*)&b_rops, 4, 0);
			delay_us(50);

			// Also send the turbine rpm value to the drive motor for ROPS detection
			TransmitCAN(MARIO_MOTOR_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
			delay_us(50);
		}
		*/
	}
	return STATE_DATA_LOGGING;
}

uint32_t DoStateDataLogging()
{
	return STATE_UART_TX;
}

// CRC-8, poly = x^8 + x^2 + x^1 + x^0, init = 0
static char CRC8_TABLE[] = {
    	(char) 0x00, (char) 0x5e, (char) 0xbc, (char) 0xe2, (char) 0x61, (char) 0x3f, (char) 0xdd, (char) 0x83,
    	(char) 0xc2, (char) 0x9c, (char) 0x7e, (char) 0x20, (char) 0xa3, (char) 0xfd, (char) 0x1f, (char) 0x41,
    	(char) 0x9d, (char) 0xc3, (char) 0x21, (char) 0x7f, (char) 0xfc, (char) 0xa2, (char) 0x40, (char) 0x1e,
    	(char) 0x5f, (char) 0x01, (char) 0xe3, (char) 0xbd, (char) 0x3e, (char) 0x60, (char) 0x82, (char) 0xdc,
    	(char) 0x23, (char) 0x7d, (char) 0x9f, (char) 0xc1, (char) 0x42, (char) 0x1c, (char) 0xfe, (char) 0xa0,
    	(char) 0xe1, (char) 0xbf, (char) 0x5d, (char) 0x03, (char) 0x80, (char) 0xde, (char) 0x3c, (char) 0x62,
    	(char) 0xbe, (char) 0xe0, (char) 0x02, (char) 0x5c, (char) 0xdf, (char) 0x81, (char) 0x63, (char) 0x3d,
    	(char) 0x7c, (char) 0x22, (char) 0xc0, (char) 0x9e, (char) 0x1d, (char) 0x43, (char) 0xa1, (char) 0xff,
    	(char) 0x46, (char) 0x18, (char) 0xfa, (char) 0xa4, (char) 0x27, (char) 0x79, (char) 0x9b, (char) 0xc5,
    	(char) 0x84, (char) 0xda, (char) 0x38, (char) 0x66, (char) 0xe5, (char) 0xbb, (char) 0x59, (char) 0x07,
    	(char) 0xdb, (char) 0x85, (char) 0x67, (char) 0x39, (char) 0xba, (char) 0xe4, (char) 0x06, (char) 0x58,
    	(char) 0x19, (char) 0x47, (char) 0xa5, (char) 0xfb, (char) 0x78, (char) 0x26, (char) 0xc4, (char) 0x9a,
    	(char) 0x65, (char) 0x3b, (char) 0xd9, (char) 0x87, (char) 0x04, (char) 0x5a, (char) 0xb8, (char) 0xe6,
    	(char) 0xa7, (char) 0xf9, (char) 0x1b, (char) 0x45, (char) 0xc6, (char) 0x98, (char) 0x7a, (char) 0x24,
    	(char) 0xf8, (char) 0xa6, (char) 0x44, (char) 0x1a, (char) 0x99, (char) 0xc7, (char) 0x25, (char) 0x7b,
    	(char) 0x3a, (char) 0x64, (char) 0x86, (char) 0xd8, (char) 0x5b, (char) 0x05, (char) 0xe7, (char) 0xb9,
    	(char) 0x8c, (char) 0xd2, (char) 0x30, (char) 0x6e, (char) 0xed, (char) 0xb3, (char) 0x51, (char) 0x0f,
    	(char) 0x4e, (char) 0x10, (char) 0xf2, (char) 0xac, (char) 0x2f, (char) 0x71, (char) 0x93, (char) 0xcd,
    	(char) 0x11, (char) 0x4f, (char) 0xad, (char) 0xf3, (char) 0x70, (char) 0x2e, (char) 0xcc, (char) 0x92,
    	(char) 0xd3, (char) 0x8d, (char) 0x6f, (char) 0x31, (char) 0xb2, (char) 0xec, (char) 0x0e, (char) 0x50,
    	(char) 0xaf, (char) 0xf1, (char) 0x13, (char) 0x4d, (char) 0xce, (char) 0x90, (char) 0x72, (char) 0x2c,
    	(char) 0x6d, (char) 0x33, (char) 0xd1, (char) 0x8f, (char) 0x0c, (char) 0x52, (char) 0xb0, (char) 0xee,
    	(char) 0x32, (char) 0x6c, (char) 0x8e, (char) 0xd0, (char) 0x53, (char) 0x0d, (char) 0xef, (char) 0xb1,
    	(char) 0xf0, (char) 0xae, (char) 0x4c, (char) 0x12, (char) 0x91, (char) 0xcf, (char) 0x2d, (char) 0x73,
    	(char) 0xca, (char) 0x94, (char) 0x76, (char) 0x28, (char) 0xab, (char) 0xf5, (char) 0x17, (char) 0x49,
    	(char) 0x08, (char) 0x56, (char) 0xb4, (char) 0xea, (char) 0x69, (char) 0x37, (char) 0xd5, (char) 0x8b,
    	(char) 0x57, (char) 0x09, (char) 0xeb, (char) 0xb5, (char) 0x36, (char) 0x68, (char) 0x8a, (char) 0xd4,
    	(char) 0x95, (char) 0xcb, (char) 0x29, (char) 0x77, (char) 0xf4, (char) 0xaa, (char) 0x48, (char) 0x16,
    	(char) 0xe9, (char) 0xb7, (char) 0x55, (char) 0x0b, (char) 0x88, (char) 0xd6, (char) 0x34, (char) 0x6a,
    	(char) 0x2b, (char) 0x75, (char) 0x97, (char) 0xc9, (char) 0x4a, (char) 0x14, (char) 0xf6, (char) 0xa8,
    	(char) 0x74, (char) 0x2a, (char) 0xc8, (char) 0x96, (char) 0x15, (char) 0x4b, (char) 0xa9, (char) 0xf7,
    	(char) 0xb6, (char) 0xe8, (char) 0x0a, (char) 0x54, (char) 0xd7, (char) 0x89, (char) 0x6b, (char) 0x35};

uint32_t crcFast(uint8_t* message, int nBytes)
{
    uint8_t data;
    uint32_t remainder = 0;

    // Divide the message by the polynomial, a byte at a time.
    for (int byte = 0; byte < nBytes; ++byte)
    {
#define CRC_WIDTH 8
        data = message[byte] ^ (remainder >> (CRC_WIDTH - 8));
        remainder = CRC8_TABLE[data] ^ (remainder << 8);
    }

    return remainder;
}

void SendLoraFrame(unsigned int id, uint8_t* data)
{
	char trame[15] = { 0 };
	memset(trame, 0x0, 15);
	// Header
	/*
	trame[0] = 0x55;
	trame[1] = 0xAA;
	trame[2] = 0x04;
	trame[3] = 0x05;
	trame[4] = 0xF1;
	trame[5] = 0xA2;
	trame[6] = 0x5B;
	trame[7] = 0x86;
	trame[8] = 0x77;
	*/
	trame[0] = 0xAA;
	trame[1] = 0x55;
	// ID

	trame[3] = (id & 0xFF00) >> 8;
	trame[2] = id & 0xFF;
	// 8-byte data
	memcpy(&trame[6], data, 8);
	// CRC
	trame[14] = crcFast((uint8_t*)trame, 15);

	// trame = { 0xAA, 0x55, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


	// char test[] = "ab";
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)test, 2, HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)trame, 15, HAL_MAX_DELAY);
	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart7, (uint8_t*)trame, 15, HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
	if (ret != HAL_OK)
	{
		// UART TX Error
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
}

void SendToLora()
{
	float data[2];
	uint32_t data_u32[2];
	uint8_t data_u8[8];

	data[0] = sensor_data.wind_speed;
	data[1] = sensor_data.wind_direction;
	// data[0] = 123.4f;
	// data[1] = 1998.20f;
	SendLoraFrame(0x101, (uint8_t*)&data);
	delay_us(20);

	data[0] = sensor_data.wheel_rpm;
	data[1] = sensor_data.vehicle_speed;
	SendLoraFrame(0x102, (uint8_t*)&data);
	delay_us(20);

	data[0] = sensor_data.rotor_rpm;
	data[1] = sensor_data.torque;
	SendLoraFrame(0x103, (uint8_t*)&data);
	delay_us(20);

	data[0] = sensor_data.loadcell;
	data[1] = 0;
	SendLoraFrame(0x104, (uint8_t*)&data);
	delay_us(20);

	data[0] = (float)sensor_data.pitch_encoder;
	data[1] = sensor_data.pitch_angle;
	SendLoraFrame(0x105, (uint8_t*)&data);
	delay_us(20);

	data_u32[0] = sensor_data.feedback_pitch_mode;
	data_u32[1] = sensor_data.feedback_mast_mode;
	SendLoraFrame(0x106, (uint8_t*)&data_u32);
	delay_us(20);

	uint32_t can_esr = CAN1->ESR;
	data_u8[0] = (can_esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
	data_u8[1] = (can_esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
	data_u8[2] = (can_esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
	data_u8[3] = (can_esr & CAN_ESR_BOFF_Msk) >> CAN_ESR_BOFF_Pos;
	data_u8[4] = (can_esr & CAN_ESR_EPVF_Msk) >> CAN_ESR_EPVF_Pos;
	data_u8[5] = (can_esr & CAN_ESR_EWGF_Msk) >> CAN_ESR_EWGF_Pos;
	data_u8[6] = data_u8[7] = 0;
	SendLoraFrame(0x107, (uint8_t*)&data_u8);
	delay_us(20);

	// Power + TSR
	data[0] = sensor_data.power;
	data[1] = CalcTSR();
	SendLoraFrame(0x108, (uint8_t*)&data_u32);
	delay_us(20);

	data_u8[0] = b_rops;
	SendLoraFrame(0x109, (uint8_t*)&data_u8);
	delay_us(20);

	data[0] = pitch_auto_target;
	data[1] = pitch_rops_target;
	// data[0] = 34.78f;
	// data[1] = 190.6f;
	SendLoraFrame(0x10C, (uint8_t*)&data);
	delay_us(20);
}

uint32_t DoStateUartTx()
{
#define UART_LORA 1
	if (flag_uart_tx_send)
	{
		flag_uart_tx_send = 0;

		if (UART_LORA)
		{
			SendToLora();
			return STATE_ACQUISITION;
		}

		// UartTxAcquisition();
		// return STATE_ACQUISITION;


		//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
		// HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		// HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		static unsigned char wind_speed_str[20] = {0};
		static unsigned char wind_direction_str[20] = {0};
		FloatToString(sensor_data.wind_speed, 2, wind_speed_str);
		FloatToString(sensor_data.wind_direction, 2, wind_direction_str);

		static unsigned char rotor_rpm_str[20] = {0};
		static unsigned char wheel_rpm_str[20] = {0};
		FloatToString(sensor_data.rotor_rpm, 2, rotor_rpm_str);
		FloatToString(sensor_data.wheel_rpm, 2, wheel_rpm_str);

		static unsigned char torque_str[20] = {0};
		static unsigned char loadcell_str[20] = {0};
		FloatToString(sensor_data.torque, 2, torque_str);
		FloatToString(sensor_data.loadcell, 2, loadcell_str);

		// Compute angle from pitch encoder value
		float pitch_angle = CalcPitchAnglePales(TRUE);
		static unsigned char pitch_angle_str[20] = {0};
		FloatToString(pitch_angle, 2, pitch_angle_str);

		static unsigned char clear_cmd[] = "\33c\e[3J";
		// if (HAL_OK != HAL_UART_Transmit(&huart2, clear_cmd, strlen((char*)clear_cmd), 1))
		if (HAL_OK != HAL_UART_Transmit(&huart7, clear_cmd, strlen((char*)clear_cmd), 1))
		{
			// UART TX Error
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}

		// float tsr = CalcTSR();
		static unsigned char tsr_str[20] = {0};
		// FloatToString(tsr, 4, tsr_str);

		// float pitch_auto_target = CalcPitchAuto();
		static unsigned char pitch_auto_target_str[20] = {0};
		// FloatToString(pitch_auto_target, 4, pitch_auto_target_str);
		// unsigned char msg[] = "Hello World! ";

		char msg[1024] = { 0 };
		sprintf(msg, "Wind Speed = %s,  Wind Direction = %s \n\rRotor rpm = %s,  wheel_rpm = %s \n\rPitch = %d,  angle = %s \n\rTorque = %s,  Loadcell = %s \n\rMast mode = %d,  Pitch mode = %d \n\rTSR = %s,  Pitch auto target = %s\n\rROPS = %d,  ROPS drive pitch = %d \n\r",
				wind_speed_str, wind_direction_str,
				rotor_rpm_str, wheel_rpm_str,
				(int)sensor_data.pitch_encoder, pitch_angle_str,
				torque_str, loadcell_str,
				(int)sensor_data.feedback_mast_mode, (int)sensor_data.feedback_pitch_mode,
				tsr_str, pitch_auto_target_str,
				(int)b_rops, (int)sensor_data.feedback_pitch_rops);
		//sprintf(msg, "Wind Speed = %d,  Wind Direction = %d \n\r", 1234, 5678);

		// char msg[] = "Hello World! \n\r";

		// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart7, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		if (ret != HAL_OK)
		{
			// UART TX Error
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}



		delay_us(200);

		uint32_t can_esr = CAN1->ESR;
		uint8_t can_rec = (can_esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
		uint8_t can_tec = (can_esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
		uint8_t can_lec = (can_esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
		uint8_t can_boff = (can_esr & CAN_ESR_BOFF_Msk) >> CAN_ESR_BOFF_Pos;
		uint8_t can_error_passive = (can_esr & CAN_ESR_EPVF_Msk) >> CAN_ESR_EPVF_Pos;
		uint8_t can_error_warning = (can_esr & CAN_ESR_EWGF_Msk) >> CAN_ESR_EWGF_Pos;

		char can_msg[512] = { 0 };
		static int test = 0;
		++test;
		sprintf(can_msg, "\n\r\n\rTEC = %d,  REC = %d  \n\rLast Error Code = %d \n\rBOFF = %d,  Error Passive = %d,  Error Warning = %d\n\r test = %d",
				can_tec, can_rec, can_lec, can_boff, can_error_passive, can_error_warning, test);

		// ret = HAL_UART_Transmit(&huart2, (uint8_t*)can_msg, strlen(can_msg), HAL_MAX_DELAY);
		ret = HAL_UART_Transmit(&huart7, (uint8_t*)can_msg, strlen(can_msg), HAL_MAX_DELAY);
		if (ret != HAL_OK)
		{
			// UART TX Error
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}

	}

	return STATE_ACQUISITION;
}

void DoStateError()
{

}

void FloatToString(float value, int decimal_precision, unsigned char* val)
{
	int integer = (int)value;
	int decimal = (int)((value - (float)integer) * (float)(pow(10, decimal_precision)));
	decimal = abs(decimal);

	sprintf((char*)val, "%d.%d", integer, decimal);
}

void SendROPSCmdCan(uint32_t rops_cmd)
{
	TransmitCAN(MARIO_ROPS_CMD, (uint8_t*)&rops_cmd, 4, 0);
	delay_us(100);
}
void SendPitchCmdCan(int nb_steps)
{
	TransmitCAN(MARIO_PITCH_CMD, (uint8_t*)&nb_steps, 4, 0);
	pitch_done = 0;
	delay_us(100);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  // current_state = STATE_INIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ExecuteStateMachine();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /*
    CAN_FilterTypeDef filter_fifo0;
        	// All common bits go into the ID register
        filter_fifo0.FilterIdHigh = MARIO_FIFO0_RX_FILTER_ID_HIGH;
        filter_fifo0.FilterIdLow = MARIO_FIFO0_RX_FILTER_ID_LOW;

        	// Which bits to compare for filter
        filter_fifo0.FilterMaskIdHigh = MARIO_FIFO0_RX_FILTER_MASK_HIGH;
        filter_fifo0.FilterMaskIdLow = MARIO_FIFO0_RX_FILTER_MASK_LOW;

        filter_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter_fifo0.FilterBank = 18; // Which filter to use from the assigned ones
        filter_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_fifo0.FilterActivation = CAN_FILTER_ENABLE;
        filter_fifo0.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
    	if (HAL_CAN_ConfigFilter(&hcan1, &filter_fifo0) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	*/
  /*
      CAN_FilterTypeDef filter_all;
      	// All common bits go into the ID register
      filter_all.FilterIdHigh = 0x0000;
      filter_all.FilterIdLow = 0x0000;

      	// Which bits to compare for filter
      filter_all.FilterMaskIdHigh = 0x0000;
      filter_all.FilterMaskIdLow = 0x0000;

      filter_all.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      filter_all.FilterBank = 18; // Which filter to use from the assigned ones
      filter_all.FilterMode = CAN_FILTERMODE_IDMASK;
      filter_all.FilterScale = CAN_FILTERSCALE_32BIT;
      filter_all.FilterActivation = CAN_FILTER_ENABLE;
      filter_all.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
  	if (HAL_CAN_ConfigFilter(&hcan1, &filter_all) != HAL_OK)
  	{
  	  Error_Handler();
  	}
  */


	CAN_FilterTypeDef sf_fifo0;
	// All common bits go into the ID register
	sf_fifo0.FilterIdHigh = MARIO_FIFO0_RX_FILTER_ID_HIGH;
	sf_fifo0.FilterIdLow = MARIO_FIFO0_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_fifo0.FilterMaskIdHigh = 0x0000;
	sf_fifo0.FilterMaskIdLow = (MARIO_FIFO0_RX_FILTER_MASK_LOW & 0x07FF);

	sf_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_fifo0.FilterBank = 2; // Which filter to use from the assigned ones
	sf_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo0.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo0.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo0) != HAL_OK)
	{
	  Error_Handler();
	}


	CAN_FilterTypeDef sf_fifo1;
	// All common bits go into the ID register
	sf_fifo1.FilterIdHigh = MARIO_FIFO1_RX_FILTER_ID_HIGH;
	sf_fifo1.FilterIdLow = MARIO_FIFO1_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_fifo1.FilterMaskIdHigh = 0x0000;
	sf_fifo1.FilterMaskIdLow = (MARIO_FIFO1_RX_FILTER_MASK_LOW & 0x07FF);

	sf_fifo1.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	sf_fifo1.FilterBank = 3; // Which filter to use from the assigned ones
	sf_fifo1.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo1.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo1.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo1.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo1) != HAL_OK)
	{
	  Error_Handler();
	}


    if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	// if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE) != HAL_OK)
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	// if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */
  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 480;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 480;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */
  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */
  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 4800;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pitch_Clock_Pin|LORA_EN_Pin|LORA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USB_RESET_Pin|USB_TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_WARNING_Pin|LED_ERROR_Pin|LED_CANA_Pin|LED_CANB_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pitch_Clock_Pin LORA_EN_Pin LORA_RESET_Pin */
  GPIO_InitStruct.Pin = Pitch_Clock_Pin|LORA_EN_Pin|LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT1_Pin LIMIT2_Pin LORA_INT_Pin Rotor_RPM_Pin
                           Wheel_RPM_Pin */
  GPIO_InitStruct.Pin = LIMIT1_Pin|LIMIT2_Pin|LORA_INT_Pin|Rotor_RPM_Pin
                          |Wheel_RPM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Pitch_Data_Pin */
  GPIO_InitStruct.Pin = Pitch_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pitch_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Mast_Data_Pin */
  GPIO_InitStruct.Pin = Mast_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mast_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Mast_Clock_Pin */
  GPIO_InitStruct.Pin = Mast_Clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mast_Clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_RESET_Pin USB_TX_EN_Pin */
  GPIO_InitStruct.Pin = USB_RESET_Pin|USB_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_CS_Pin */
  GPIO_InitStruct.Pin = LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_WARNING_Pin LED_ERROR_Pin LED_CANA_Pin LED_CANB_Pin
                           LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED_WARNING_Pin|LED_ERROR_Pin|LED_CANA_Pin|LED_CANB_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_Pin PB1_Pin */
  GPIO_InitStruct.Pin = PB2_Pin|PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

