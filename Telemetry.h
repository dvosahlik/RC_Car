// Telemetry.h

#ifndef _TELEMETRY_h
#define _TELEMETRY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

typedef struct {
	char RPM_left : 1;
	char RPM_right : 1;
	char loop_time : 1;
	char telemtry_data : 1;
	char vypocetni_cas :1;
	} debug_messages_t;
	
	
	typedef struct {
		char turn_off_rpm : 1;
		char turn_off_all : 1;
	} communication_data_t;

	typedef struct {
		char identifier;
		double data;
		uint32_t time;
	} telemetry_data_t;

extern volatile debug_messages_t debug_messages;

char set_up_telemetry();

void print_telemetry_data(telemetry_data_t* data, int length);

char check_buffer();


extern double ENGINE_RPM_FEEDBACK_GAIN;
extern double ENGINE_RPM_GAIN;
extern communication_data_t com_data;
extern double const2;
extern double GYRO_ACCEL_FEEDBACK_GAIN;
extern double ENGINE_FEEDBACK_GAIN_LIMITER;
extern double throttle_in;
extern uint32_t throttle_start;
extern uint16_t THROTTLE_DURATION;
extern uint32_t next_test;

#endif

