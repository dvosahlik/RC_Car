// 
// 
// 

#include "Gain.h"
#include "Types.h"

double gain_output(double input, signal_block* block)
{
	double gain = *((double*)(block->pointer_on_data));
	return input*gain;
}

char gain(double gain, signal_block* block)
{
	block->serial_signal_output = gain_output;
	double* k = (double*)malloc(sizeof(double));
	*k = gain;
	block->pointer_on_data = k;
}


char var_gain(double* gain, signal_block* block)
{
	block->serial_signal_output = gain_output;
	block->pointer_on_data = gain;
}

