// 
// 
// 

#include "Gain.h"
#include "Types.h"

/************************************************************************/
/* Interface function used to return the output of the Gain signal block                                                                     */
/************************************************************************/
double gain_output(double input, signal_block* block)
{
	double gain = *((double*)(block->pointer_on_data));
	return input*gain;
}

/************************************************************************/
/* constructor function that generates the Gain signal block                                                                    */
/************************************************************************/
char gain(double gain, signal_block* block)
{
	block->serial_signal_output = gain_output;
	double* k = (double*)malloc(sizeof(double));
	*k = gain;
	block->pointer_on_data = k;
}

/************************************************************************/
/* constructor function that generates the Gain signal block. In this overload the gain is adjustable in runtime                                                                     */
/************************************************************************/
char var_gain(double* gain, signal_block* block)
{
	block->serial_signal_output = gain_output;
	block->pointer_on_data = gain;
}

