// Telemetrie.h

#ifndef _TELEMETRIE_h
#define _TELEMETRIE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


char send_bytes(char* ptr, int length);

char send_bytes_DMA(char* ptr, int length);

char read_bytes_DMA(char* ptr, int length);

extern volatile char sending_tel;
extern char read_buffer[50];
extern char zpracuj_buffer;
extern int read_index;


#endif

