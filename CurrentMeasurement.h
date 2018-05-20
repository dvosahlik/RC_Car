// CurrentMeasurement.h

#ifndef _CURRENTMEASUREMENT_h
#define _CURRENTMEASUREMENT_h

#define CURRENT_LEFT_ADC_CHANELL		ADC_CHANNEL_0			//ANALOG pin A7
#define CURRENT_LEFT_NULL_ADC_CHANELL	ADC_CHANNEL_1			//ANALOG pin A6
#define CURRENT_RIGHT_ADC_CHANELL		ADC_CHANNEL_3			//ANALOG pin A4
#define CURRENT_RIGHT_NULL_ADC_CHANELL	ADC_CHANNEL_2			//ANALOG pin A5

#define MOTOR_DATA_LENGTH 500			//musí být násobek 4
#define MIN_ALLOWED_FREQUENCY 400	//Hz
#define MAX_ALLOWED_FREQUENCY 5000  //Hz
#define MIN_ALLOWED_PERIOD (float)1/(float)MAX_ALLOWED_FREQUENCY*(float)1000000		//minimální odstup v us
#define ADC_SAMPLING_PERIOD 39		//us
#define ADC_SAMPLING_PERIOD_HALF ADC_SAMPLING_PERIOD/2		//us
#define MOTOR_MAX_ITERATIONS 1
#define ADC_STEP	0.8056640625	//mV
#define MOTOR_HYSTERESIS_OFFSET MOTOR_HYSTERESIS_OFFSET_VOLTS/ADC_STEP

#define ADC_AVREF						3.3						//3.3V
#define ADC_MAX_VALUE					0xfff					// 12 bit resolution

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

char set_up_adc();

char get_motor_RPM(char left, volatile int*value);

char get_motor_currents(volatile int32_t* value_l, volatile int32_t* value_r);

char fill_buffer(char left);

extern uint8_t MOTOR_HYSTERESIS_OFFSET_VOLTS;			//mV
extern uint8_t MOTOR_HYSTERESIS_OFFSET_DURATION;

extern int test_adc;

extern volatile float zero_cross[MOTOR_DATA_LENGTH];

#endif

