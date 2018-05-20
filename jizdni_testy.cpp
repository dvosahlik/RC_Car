// 
// 
// 

#include "Telemetry.h"
#include "Arduino.h"
#include "jizdni_testy.h"

int16_t throttle_old;
uint32_t throttle_time;

uint32_t zbyvajici_cas()
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


char zataceni(inputs_t* inputs)
{
	uint32_t cas = zbyvajici_cas();
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

char akcelerace(inputs_t* inputs)
{
	uint32_t cas = zbyvajici_cas();
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

char klickovani(inputs_t* inputs)
{
	uint32_t cas = zbyvajici_cas();
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

char testovaci_vstupy(inputs_t* inputs)
{
	char testy = next_test % 3;
	switch (testy)
	{
		case 0 :
			akcelerace(inputs);
		break;
		case 1:
			zataceni(inputs);
		break;
		case 2:
			klickovani(inputs);
		break;
	}
}