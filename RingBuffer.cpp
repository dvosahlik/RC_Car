// 
// 
// 

#include "RingBuffer.h"

char circBufFull(circBuf_t *c)
{
	// next is where head will point to after this write.
	int next = c->head + 1;
	if (next >= c->maxLen)
	next = 0;

	if (next == c->tail) // check if circular buffer is full
	{
		return 1;
	}
	return 0;  // return success to indicate successful push.
}


char circBufPush(circBuf_t *c, int16_t data)
{
	// next is where head will point to after this write.
	int next = c->head + 1;
	if (next >= c->maxLen)
		next = 0;

	if (next == c->tail) // check if circular buffer is full
	{
		int nexttail = c->tail + 1;
		if (next >= c->maxLen)
		next = 0;
		c->tail = nexttail;
	}

	c->buffer[c->head] = data; // Load data and then move
	c->head = next;            // head to next data offset.
	return 1;  // return success to indicate successful push.
}

char circ_buf_push_burst(circBuf_t *c, int16_t* data, int length)
{
	// next is where head will point to after this write.
	int next = c->head + 1;
	for (int i = 0; i < length;i++)
	{
		if (next >= c->maxLen)
		next = 0;

		if (next == c->tail) // check if circular buffer is full
		{
			int nexttail = c->tail + 1;
			if (nexttail >= c->maxLen)
				nexttail = 0;
			c->tail = nexttail;
		}

		c->buffer[c->head] = data[i]; // Load data and then move
		c->head = next;            // head to next data offset.
		next++;
	}
	return 1;  // return success to indicate successful push.
}


char circBufPop(circBuf_t *c, int16_t *data)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (c->head == c->tail) // check if circular buffer is empty
	return 0;          // and return with an error

	// next is where tail will point to after this read.
	int next = c->tail + 1;
	if(next >= c->maxLen)
	next = 0;

	*data = c->buffer[c->tail]; // Read data and then move
	c->tail = next;             // tail to next data offset.
	return 1;  // return success to indicate successful push.
}

char circ_buf_on_index(circBuf_t *c, int index, int16_t *data)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (c->head == c->tail) // check if circular buffer is empty
	return 0;          // and return with an error

	// next is where tail will point to after this read.
	int next = c->tail + index;
	if(next >= c->maxLen)
	next = c->tail + index - c->maxLen;

	*data = c->buffer[next]; // Read data and then move
	return 1;  // return success to indicate successful push.
}


char circBufFull32(circBuf32_t *c)
{
	// next is where head will point to after this write.
	int next = c->head + 1;
	if (next >= c->maxLen)
	next = 0;

	if (next == c->tail) // check if circular buffer is full
	{
		return 1;
	}
	return 0;  // return success to indicate successful push.
}


char circBufPush32(circBuf32_t *c, uint32_t data)
{
	// next is where head will point to after this write.
	int next = c->head + 1;
	if (next >= c->maxLen)
	next = 0;

	if (next == c->tail) // check if circular buffer is full
	{
		int nexttail = c->tail + 1;
		if (next >= c->maxLen)
		next = 0;
		c->tail = nexttail;
	}

	c->buffer[c->head] = data; // Load data and then move
	c->head = next;            // head to next data offset.
	return 1;  // return success to indicate successful push.
}

char circBufPop32(circBuf32_t *c, uint32_t *data)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (c->head == c->tail) // check if circular buffer is empty
	return 0;          // and return with an error

	// next is where tail will point to after this read.
	int next = c->tail + 1;
	if(next >= c->maxLen)
	next = 0;

	*data = c->buffer[c->tail]; // Read data and then move
	c->tail = next;             // tail to next data offset.
	return 1;  // return success to indicate successful push.
}

char circ_buf_on_index32(circBuf32_t *c, int index, uint32_t *data)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (c->head == c->tail) // check if circular buffer is empty
	return 0;          // and return with an error

	// next is where tail will point to after this read.
	int next = c->tail + index;
	if(next >= c->maxLen)
	next = c->tail + index - c->maxLen;

	*data = c->buffer[next]; // Read data and then move
	return 1;  // return success to indicate successful push.
}