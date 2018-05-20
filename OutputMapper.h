// OutputMapper.h

#include "Types.h"

#ifndef _OUTPUTMAPPER_h
#define _OUTPUTMAPPER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define MINIMUM_VYSTUPNIHO_SIGNALU 900
#define STRED_VYSTUPNIHO_SIGNALU_MOTOR 1410
#define STRED_VYSTUPNIHO_SIGNALU_SERVO 1500
#define MAXIMUM_VYSTUPNIHO_SIGNALU 2100

char scale_throttle_output(uint8_t percent, uint8_t percent_left);

char map_to_steering(int steering);

char map_to_throttle(torque_vector* throttle);

 char scale_steering_output(uint8_t percent);
#endif

