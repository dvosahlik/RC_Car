// signal_path_serial.h

#ifndef _SIGNAL_PATH_SERIAL_h
#define _SIGNAL_PATH_SERIAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Types.h"


typedef struct {
		signal_block** blocks;
		int size_used;
		int size;
	} serial_signal_path_t;

double get_output(double input, serial_signal_path_t* path);

char register_member(signal_block* member, serial_signal_path_t* path);

#endif

