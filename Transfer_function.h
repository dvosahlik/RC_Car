// Transfer_function.h

#ifndef _TRANSFER_FUNCTION_h
#define _TRANSFER_FUNCTION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Types.h"

typedef struct {
	double *nominator;
	double *denominator;
	int nominator_size;
	int denominator_size;
	} s_function;
	
		typedef struct {
			double *nominator;			//nominator[0] = z^n, nominator[1] = z^(n - 1), ...
			double *denominator;
			int nominator_size;
			int denominator_size;
		} z_function;
	

char Transfer_function(z_function* fce, signal_block* block);


#endif

