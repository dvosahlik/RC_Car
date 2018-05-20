// RPM_measurement.h

#ifndef _RPM_MEASUREMENT_h
#define _RPM_MEASUREMENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


char set_up_RPS();

double get_RPS_right();

double get_RPS_left();	

#endif

