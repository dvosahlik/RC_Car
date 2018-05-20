// Gain.h

#ifndef _GAIN_h
#define _GAIN_h

#include "Types.h"
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
char gain(double gain, signal_block* block);

char var_gain(double* gain, signal_block* block);


#endif

