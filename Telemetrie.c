// 
// 
// 

#include "Telemetrie.h"
#include "Arduino.h"
#include <string.h>


char send_char(char to_send)
{
	while ((USART3->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY);
	USART3->US_THR = to_send;
}
char (*read_done)(char*, int) = NULL;

char (*send_done)() = NULL;


char register_send_done(char (*callback)())
{
	send_done = callback;
}

char send_bytes(char* ptr, int length)
{
	for (int i = 0; i < length;i++)
	{
		send_char(ptr[i]);
	}
}

volatile char sending_tel = 0;
volatile char* buffer[128];
char read_buffer[50];
int read_index = -1;

char send_bytes_DMA(char* ptr, int length)
{
	if (sending_tel)
		return 0;
	sending_tel = 1;
	USART3->US_IDR &= ~(US_IDR_ENDRX | US_IDR_ENDTX);
	USART3->US_IER |= US_IER_ENDTX;
	memcpy(buffer, ptr, length);
	USART3->US_TPR = buffer;
	USART3->US_TCR = length;
	USART3->US_TNCR = 0;
	USART3->US_TNPR = 0;
	USART3->US_PTCR &= ~(US_PTCR_TXTDIS);
	USART3->US_PTCR |= US_PTCR_TXTEN;
	if (read_index == -1)
		read_bytes_DMA(read_buffer + read_index, 1);
	return 1;
}

volatile char receiving = 0;

char register_read_done(char (*callback)(char*, int))
{
	read_done = callback;
}


char zpracuj_buffer = 0;

char *ptr_read = read_buffer;
int length_read = 1;

char read_bytes_DMA(char* ptr, int length)
{
	if (receiving == 1)
		return;
	receiving = 1;
	ptr_read = ptr;
	length_read = length;
	USART3->US_IDR &= ~(US_IDR_ENDRX | US_IDR_ENDTX);
	USART3->US_IER |= US_IER_ENDRX;
	USART3->US_RPR = ptr;
	USART3->US_RCR = length;
	USART3->US_PTCR &= ~(US_PTCR_RXTDIS);
	USART3->US_PTCR |= US_PTCR_RXTEN;
}

static char read_first = 1;

void USART3_Handler(void)
{
	int32_t cs = USART3->US_CSR;
	if ((cs & US_CSR_ENDRX) == US_CSR_ENDRX)
	{
		USART3->US_RCR = 1;
		USART3->US_IDR = ~(US_IDR_ENDTX);
		USART3->US_IER &= ~(US_IER_ENDRX);
		USART3->US_PTCR &= ~(US_PTCR_RXTEN);
		USART3->US_PTCR |= US_PTCR_RXTDIS;
		ptr_read[length_read - 1] = USART3->US_RHR;
		if (read_done)
			read_done(ptr_read, length_read);
		receiving = 0;
		if (!zpracuj_buffer)
		{
			if (read_index >= 0 && read_buffer[read_index] == 0x78)
			{
				zpracuj_buffer = 1;
			}
			else
			{
				if (read_index == 0 && read_first)
				{
					read_first = 0;
				}
				else if (read_index == 0)
				{
					read_buffer[0] = read_buffer[49];
				}
				read_index++;
				char *next = read_buffer;
				next += read_index;
				read_bytes_DMA(next, 1);
				
				return;
			}
		}
		read_bytes_DMA(read_buffer + 49, 1);
	}
	if ((cs & US_CSR_ENDTX) == US_CSR_ENDTX)
	{
		USART3->US_TCR = 10;
		USART3->US_IDR = ~(US_IDR_ENDRX);
		USART3->US_IER &= ~(US_IER_ENDTX);
		USART3->US_PTCR &= ~(US_PTCR_TXTEN);
		USART3->US_PTCR |= US_PTCR_TXTDIS;
		sending_tel = 0;
		
	}
	
}