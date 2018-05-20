/*
 * Types.h
 *
 * Created: 05.02.2017 19:25:33
 *  Author: David
 */ 


#ifndef TYPES_H_
#define TYPES_H_



typedef struct {
	int left_wheel;
	int right_wheel;
} torque_vector;

typedef struct signal_block signal_block;
	
struct signal_block {
	void* pointer_on_data;
	double (*serial_signal_output)(double, signal_block*);
	double minimum;
	char use_minimum = 0;
	double maximum;
	char use_maximum = 0;
	} ;
	



#endif /* TYPES_H_ */