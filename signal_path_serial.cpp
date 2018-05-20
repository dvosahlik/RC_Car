// 
// 
// 

#include <stdlib.h>

#include "signal_path_serial.h"
#include "Types.h"

#define SIGNAL_PATH_INCREMENT 2



char register_member(signal_block* member, serial_signal_path_t* path)
{
	if (path->size == 0)
	{
		(path->blocks) = (signal_block**)malloc(sizeof(signal_block*)*SIGNAL_PATH_INCREMENT);
		(path->size) += SIGNAL_PATH_INCREMENT;
	}
	else if (path->size_used == path->size)
	{
		(path->blocks) = (signal_block**)realloc(path->blocks, path->size + SIGNAL_PATH_INCREMENT*sizeof(signal_block*));
		(path->size) += SIGNAL_PATH_INCREMENT;
	}
	(path->blocks[path->size_used]) = member;
	(path->size_used) = path->size_used + 1;
}

double get_output(double input, serial_signal_path_t* path)
{
	double result = input;
	for (int i = 0; i < path->size_used;i++)
	{
		result = (path->blocks[i])->serial_signal_output(result, (path->blocks[i]));
	}
	return result;
}