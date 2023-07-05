

#include "motor_control.h"

#include <stdlib.h>

#include "main.h"
#include "can.h"
#include "sensors.h"
#include "pitch.h"

#define PITCH_UPDATE_DEG_THRESHOLD 0.25f

void DoPitchControl()
{
	if (sensor_data.feedback_pitch_mode == MOTOR_MODE_AUTOMATIC)
	{
		float new_pitch_target = CalcPitchAuto();
		pitch_auto_target = new_pitch_target;
		// VerifyPitchCmd();

		// if (abs(new_pitch_target - pitch_auto_target) > PITCH_UPDATE_THRESHOLD)
		if (abs(new_pitch_target - sensor_data.pitch_angle) > PITCH_UPDATE_DEG_THRESHOLD)
		{
			float delta_angle_pales = CalcPitchAnglePales(TRUE) - new_pitch_target;

	#define MIN_ERROR_ANGLE 0.1f
			if (abs(delta_angle_pales) > 0.1f)
			{
				if (pitch_done)
				{
					// Small delay inbetween commands for smoothness
					static uint32_t pitch_done_counter = 0;
					++pitch_done_counter;

	#define PITCH_DONE_WAIT_NUM 6
					if (pitch_done_counter >= PITCH_DONE_WAIT_NUM)
					{
						pitch_done_counter = 0;
						SendPitchAngleCmd(new_pitch_target);
					}
				}

			}
		}
	}
}

void DoMastControl()
{

}
