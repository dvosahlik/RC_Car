// jizdni_testy.h

#ifndef _JIZDNI_TESTY_h
#define _JIZDNI_TESTY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

typedef struct {
	int16_t throttle;
	int16_t steering;
	} inputs_t;

char test_inputs(inputs_t* inputs);
uint32_t remaining_test_time();

#endif

