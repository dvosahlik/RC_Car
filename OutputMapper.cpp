// 
// 
// 

#include "OutputMapper.h"
#include "limits.h"
#include "Variables.h"

#define MAX_SCALE 100;
static char throttle_scale = 100;
static char throttle_scale_left = 100;
static char steering_scale = 100;

static uint32_t map_to_one_output(int value, uint8_t scale, char servo)
{
	float ss = scale/(float)MAX_SCALE;
	uint32_t stred = servo ? STRED_VYSTUPNIHO_SIGNALU_SERVO : STRED_VYSTUPNIHO_SIGNALU_MOTOR;
	uint32_t result = value*ss + stred;
	if (result < MINIMUM_VYSTUPNIHO_SIGNALU)
		result = stred;
	else if (result > MAXIMUM_VYSTUPNIHO_SIGNALU)
		result = stred;
	return result;
}

char map_to_throttle(torque_vector* throttle)
{
	throttle->left_wheel = throttle_scale_left/(float)100*throttle->left_wheel;
	throttle->left_wheel = map_to_one_output(throttle->left_wheel, throttle_scale, 0);
	throttle->right_wheel = map_to_one_output(throttle->right_wheel, throttle_scale, 0);
	THROTTLE_REGISTER_LB = throttle->left_wheel;// Throttle;
	THROTTLE_REGISTER_RB = throttle->right_wheel;//Throttle;
	return 1;
}

char map_to_steering(int steering)
{
	STEER_REGISTER = map_to_one_output(steering, steering_scale, 1);
}

char scale_throttle_output(uint8_t percent, uint8_t percent_left)
{
	throttle_scale = percent;
	return 1;
}

char scale_steering_output(uint8_t percent)
{
	steering_scale = percent;
	return 1;
}
