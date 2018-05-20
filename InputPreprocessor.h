// InputPreprocessor.h

#ifndef _INPUTPREPROCESSOR_h
#define _INPUTPREPROCESSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define MAX_VYCHYLKA_AKCNIHO_ZASAHU 500

char set_up_inputs();

int get_steering();

int get_throttle();

#endif

