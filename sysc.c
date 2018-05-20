// 
// 
// 

#include "syscalls.h"
#include "sysc.h"
#include "Arduino.h"
#include "Telemetrie.h"

#include <stdio.h>
#include <stdarg.h>



extern int _write(int file, char *ptr, int len )
{
	send_bytes_DMA(ptr, len);
}


extern int _read(int file, char *ptr, int len)
{
}
