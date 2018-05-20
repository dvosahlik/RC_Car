// RingBuffer.h

#ifndef _RINGBUFFER_h
#define _RINGBUFFER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


typedef struct {
	uint16_t* buffer;
	int head;
	int tail;
	int maxLen;
} circBuf_t;

typedef struct {
	uint32_t* buffer;
	int head;
	int tail;
	int maxLen;
} circBuf32_t;


char circBufPush(circBuf_t *c, int16_t data);

char circBufPop(circBuf_t *c, int16_t *data);

char circBufFull(circBuf_t *c);

char circ_buf_on_index(circBuf_t *c, int index, int16_t *data);

char circBufPush32(circBuf32_t *c, uint32_t data);

char circBufPop32(circBuf32_t *c, uint32_t *data);

char circBufFull32(circBuf32_t *c);

char circ_buf_on_index32(circBuf32_t *c, int index, uint32_t *data);

char circ_buf_push_burst(circBuf_t *c, int16_t* data, int length);
#endif

