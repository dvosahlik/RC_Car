// syscalls.h

#ifndef _SYSCALLS_h
#define _SYSCALLS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


extern int was_write;

#endif

