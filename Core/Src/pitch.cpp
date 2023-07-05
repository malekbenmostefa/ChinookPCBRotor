

#include "pitch.h"
#include "main.h"

#include <stdio.h>
#include <cstdlib>

#include "chinook_can_ids.h"


#define KNOTS_TO_MS 0.514444f
// #define PALE_RADIUS 0.918f
#define PALE_RADIUS 0.89f
#define ENCODER_TO_PALES_RATIO (3.0f / 2.0f)



float BoundAngleSemiCircle(float angle);


float CalcDeltaPitch(uint32_t reference_point);



float BoundAngleSemiCircle(float angle)
{
	if (angle < -180.0f)
		return angle + 360.0f;
	else if (angle > 180.0f)
		return angle - 360.0f;
	else
		return angle;
}


float CalcDeltaPitch(uint32_t reference_point)
{
	int32_t delta_pitch = (sensor_data.pitch_encoder - reference_point);
	uint32_t abs_delta_pitch = abs(delta_pitch);

	// Adjust the pitch value if greater than half distance around full circle
	if (abs_delta_pitch >= HALF_MAX_VALUE)
	{
		if (delta_pitch < 0)
			delta_pitch = (MAX_PITCH_VALUE - abs_delta_pitch);
		else
			delta_pitch = -(MAX_PITCH_VALUE - abs_delta_pitch);
	}

	return delta_pitch;
}

float CalcPitchAnglePales(uint8_t bound_angle)
{
	float delta_pitch = CalcDeltaPitch(PITCH_ABSOLUTE_ZERO);

	static const float PITCH_TO_ANGLE_RATIO = ENCODER_TO_PALES_RATIO * (360.0f / (float)MAX_PITCH_VALUE);
	float pitch_angle = (float)delta_pitch * PITCH_TO_ANGLE_RATIO;

	// Bound angle between -180 and 180 degrees
	if (bound_angle)
		pitch_angle = BoundAngleSemiCircle(pitch_angle);

	// float bounded_pitch_angle = BoundAngleSemiCircle(pitch_angle);

	return pitch_angle;
}

float CalcPitchAnglePales(uint32_t abs_target, uint8_t bound_angle)
{
	// float delta_pitch = CalcDeltaPitch(PITCH_ABSOLUTE_ZERO);
	int32_t delta_pitch = (abs_target - PITCH_ABSOLUTE_ZERO);

	static const float PITCH_TO_ANGLE_RATIO = ENCODER_TO_PALES_RATIO * (360.0f / (float)MAX_PITCH_VALUE);
	float pitch_angle = (float)delta_pitch * PITCH_TO_ANGLE_RATIO;

	// Bound angle between -180 and 180 degrees
	if (bound_angle)
		pitch_angle = BoundAngleSemiCircle(pitch_angle);

	// float bounded_pitch_angle = BoundAngleSemiCircle(pitch_angle);

	return pitch_angle;
}


float CalcTSR()
{
	static const float RPM_TO_RADS = 2.0f * PI / 60.0f;
	float rotor_speed_omega = RPM_TO_RADS * sensor_data.rotor_rpm;

	float wind_speed_ms = KNOTS_TO_MS * sensor_data.wind_speed;

	if (abs(wind_speed_ms) < MIN_EPSILON)
		return 0.0f;
	float tsr = (PALE_RADIUS * rotor_speed_omega) / wind_speed_ms;
	if (tsr < MIN_EPSILON)
		return 0.0f;
	return tsr;
}

float CalcPitchAuto()
{
#define ALGO_C11 1
#define ALGO_C10 0

	float tsr = CalcTSR();

	if (ALGO_C10)
	{
		float tsr2 = tsr * tsr;
		float tsr3 = tsr2 * tsr;
		float tsr4 = tsr2 * tsr2;
		float tsr5 = tsr4 * tsr;
		float tsr6 = tsr3 * tsr2;

		float pitch_target = -0.00361082f *  tsr6 +
							  0.12772891f *  tsr5 -
							  1.76974117f *  tsr4 +
							  11.90368681f * tsr3 -
							  38.19438424f * tsr2 +
							  43.63402687f * tsr +
							  4.55041087;
		return pitch_target;
	}
	else if (ALGO_C11)
	{
		float tsr2 = tsr * tsr;
		float tsr3 = tsr2 * tsr;
		float tsr4 = tsr2 * tsr2;
		float tsr5 = tsr4 * tsr;
		float tsr6 = tsr4 * tsr2;
		float tsr7 = tsr6 * tsr;
		float tsr8 = tsr6 * tsr2;

		float pitch_target = 531.001 -
						   502.712 * tsr  -
						   221.383 * tsr2 +
						   576.857 * tsr3 -
						   392.517 * tsr4 +
						   145.522 * tsr5 -
						   33.6882 * tsr6 +
						   5.09176 * tsr7 -
						   0.503931 * tsr8;

		return pitch_target;
	}
}

void VerifyPitchTargetCmd(uint32_t target_pitch_abs)
{
#define TARGET_MAX_ERROR 10000
	if (abs(sensor_data.pitch_encoder - PITCH_ABSOLUTE_ZERO) > TARGET_MAX_ERROR)
	{
		SendPitchTargetCmd(PITCH_ABSOLUTE_ZERO);
	}
}


void VerifyRopsCmd()
{
#define MAX_ERROR_ROPS 10000
	if (abs(sensor_data.pitch_encoder - PITCH_ABSOLUTE_ROPS) > MAX_ERROR_ROPS)
	{
		SendPitchROPSCmd();
	}
}

void SendPitchTargetCmd(uint32_t target_pitch_abs)
{
	// Negative because we want pitch to ROPS (but we compute the other direction)
	float delta_pitch = -CalcDeltaPitch(target_pitch_abs);

	static const float pitch_to_angle = 360.0f / MAX_PITCH_VALUE;
	float delta_angle_encoder = delta_pitch * pitch_to_angle;

	static const float angle_mov_per_step_inv = 293.89f / 1.8f;
	int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);
	// static const float angle_mov_per_step_inv = 293.89f / 1.8f;
	// int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);

	if(abs(nb_steps) > MAX_STEPS_PER_CMD)
	{
		if(nb_steps < 0)
			nb_steps = -MAX_STEPS_PER_CMD;
		else
			nb_steps = MAX_STEPS_PER_CMD;
	}

	if(pitch_done)
	{

		// uint32_t nb_steps_cmd = (int)
		/*
		if (sensor_data.feedback_pitch_rops != 1)
		{
			uint32_t rops_cmd = ROPS_ENABLE;
			SendROPSCmdCan(rops_cmd);
		}
		*/


		SendPitchCmdCan(nb_steps);
	}
}

void SendPitchROPSCmd()
{
	// Negative because we want pitch to ROPS (but we compute the other direction)
	float delta_pitch = -CalcDeltaPitch(PITCH_ABSOLUTE_ROPS);

	static const float pitch_to_angle = 360.0f / MAX_PITCH_VALUE;
	float delta_angle_encoder = delta_pitch * pitch_to_angle;

	static const float angle_mov_per_step_inv = 293.89f / 1.8f;
	int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);
	// static const float angle_mov_per_step_inv = 293.89f / 1.8f;
	// int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);

	if(abs(nb_steps) > MAX_STEPS_PER_CMD)
	{
		if(nb_steps < 0)
			nb_steps = -MAX_STEPS_PER_CMD;
		else
			nb_steps = MAX_STEPS_PER_CMD;
	}

	if(pitch_done)
	{
		// uint32_t nb_steps_cmd = (int)
		if (sensor_data.feedback_pitch_rops != 1)
		{
			uint32_t rops_cmd = ROPS_ENABLE;
			SendROPSCmdCan(rops_cmd);
		}

		SendPitchCmdCan(nb_steps);
	}
}

void SendPitchAngleCmd(float target_pitch)
{
	// Bounds checking
	// if (target_angle_pales < 180.0f || target_angle_pales > 180.0f)
	//	return;
	float target_angle_pales = BoundAngleSemiCircle(target_pitch);

	// Calculate current pitch angle from ABSOLUTE ZERO
	// float current_pitch_angle_pales = (3.0f / 2.0f) * pitch_to_angle(current_pitch);
	float angle_pales = CalcPitchAnglePales(FALSE);
	angle_pales = BoundAngleSemiCircle(angle_pales);

	// Calculate the delta pitch angle for the stepper motor
	float delta_angle_pales = target_angle_pales - angle_pales;
	delta_angle_pales = BoundAngleSemiCircle(delta_angle_pales);

	static const float angle_pales_to_encoder_angle = (2.0f / 3.0f);
	float delta_angle_encoder = angle_pales_to_encoder_angle * delta_angle_pales;

	// Convert the target angle to stepper steps
	// static const float angle_mov_per_step = 1.8f / 293.89f;
	// int nb_steps = (int)(delta_pitch_angle_encodeur / angle_mov_per_step);
	static const float angle_mov_per_step_inv = 293.89f / 1.8f;
	int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);

	// Set a maximum to the number of steps so that we don't overshoot too much
	// Plus, its safer in case of angle error
	//if(abs(nb_steps) > 300) nb_steps = 300;
	//nb_steps *= -1;


	if(abs(nb_steps) > MAX_STEPS_PER_CMD)
	{
		if(nb_steps < 0)
			nb_steps = -MAX_STEPS_PER_CMD;
		else
			nb_steps = MAX_STEPS_PER_CMD;
	}
	// nb_steps *= -1;
	// TODO: Add checks and validation of steps

	// Send the command to the stepper drive
	// Only send cmd if stepper has finished last command
	// Stepper drive will notice us when done with the pitch_done CAN message.
	//if(true || pitch_done)
	if(pitch_done)
	{
		/*
		if (sensor_data.feedback_pitch_rops != 1)
		{
			uint32_t rops_cmd = ROPS_ENABLE;
			SendROPSCmdCan(rops_cmd);
		}
		*/

		SendPitchCmdCan(nb_steps);
	}
}
