// 
// 
// 

#include "Telemetry.h"
#include "Arduino.h"
#include "jizdni_testy.h"

int16_t throttle_old;
uint32_t throttle_time;

/************************************************************************/
/* returns the remaining drive test time. The unit is in drive test if greater then 0                                                                     */
/************************************************************************/
uint32_t remaining_test_time()
{
	if(throttle_old != throttle_in)
	{
		throttle_time = millis();
		throttle_old = throttle_in;
	}
	if (millis() - throttle_time <= THROTTLE_DURATION)
	{
		return millis() - throttle_time;
	}
	return 0;
}

/************************************************************************/
/* get inputs for the steering test.                                                                     */
/************************************************************************/
char steering_test(inputs_t* inputs)
{
	uint32_t cas = remaining_test_time();
	if (cas)
	{
		double procento = cas*(double)100/THROTTLE_DURATION;
		if (procento < 100)
		{
			inputs->steering = 500;
		}
		if (procento > 10)
		{
			inputs->throttle = throttle_in;
		}
	}
}

/************************************************************************/
/* returns the inputs for the acceleration tests                                                                     */
/************************************************************************/
char acceleration_test(inputs_t* inputs)
{
	uint32_t cas = remaining_test_time();
	if (cas)
	{
		double procento = cas*(double)100/THROTTLE_DURATION;
		if (procento < 100)
		{
			inputs->steering = 0;
			inputs->throttle = throttle_in;
		}
	}
}

/************************************************************************/
/* returns the inputs fo the looping tests.                                                                     */
/************************************************************************/
char looping_test(inputs_t* inputs)
{
	uint32_t cas = remaining_test_time();
	if (cas)
	{
		inputs->throttle = throttle_in;
		double procento = cas*(double)100/THROTTLE_DURATION;
		if (procento < 20)
		{
			inputs->steering = 300;
		}
		else if (procento < 40)
		{
			inputs->steering = -300;
		}
		else if (procento < 60)
		{
			inputs->steering = 300;
		}
		else if (procento < 80)
		{
			inputs->steering = -300;
		}
		else if (procento < 100)
		{
			inputs->steering = 300;
		}
	}
}

/************************************************************************/
/* returns the test inputs when in test mode.                                                                     */
/************************************************************************/
char test_inputs(inputs_t* inputs)
{
	char testy = next_test % 3;
	switch (testy)
	{
		case 0 :
			acceleration_test(inputs);
		break;
		case 1:
			steering_test(inputs);
		break;
		case 2:
			looping_test(inputs);
		break;
	}
}