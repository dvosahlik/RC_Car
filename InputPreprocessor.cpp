//
//
//

#include "InputPreprocessor.h"
#include "Variables.h"
#include "OutputMapper.h"
#include "limits.h"
#include "Types.h"
#include "Transfer_function.h"
#include "signal_path_serial.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define TRIMM_DATA_LENGTH 256
#define THROTTLE_MIN_COUNT 950
#define THROTTLE_MAX_COUNT 2100

void SteerDownOccured();
void ThrotleDownOccured();

uint32_t throtleCount, throtleCountNew;
uint32_t steerCount, steerCountNew;
char steerUp, throtleUp;
uint32_t stred_vstupniho_signalu_servo = 0;
uint32_t stred_vstupniho_signalu_motor = 0;
uint32_t max_vstupni_sig_motor = 0;
uint32_t max_vstupni_sig_servo = 0;
uint32_t min_vstupni_sig_motor = 0;
uint32_t min_vstupni_sig_servo = 0;
uint32_t servo_trim_data[TRIMM_DATA_LENGTH];
uint16_t servo_trimmed = 0;
uint32_t throttle_trim_data[TRIMM_DATA_LENGTH];
uint16_t throttle_trimmed = 0;


serial_signal_path_t input_filter; 

char register_filter();

void SteerUpOccured()
{
	attachInterrupt(STEERING_INPUT_PIN, SteerDownOccured, FALLING);
	steerUp = 0;
	steerCountNew = micros();
}

void SteerDownOccured()
{
	steerCount = micros() - steerCountNew;
	if (steerCount != 0 && steerCount < MAXIMUM_VYSTUPNIHO_SIGNALU && servo_trimmed < TRIMM_DATA_LENGTH)
	{
		servo_trim_data[servo_trimmed] = steerCount;
		servo_trimmed++;
	}
	if (steerCount < min_vstupni_sig_servo || min_vstupni_sig_servo == 0)
	{
		min_vstupni_sig_servo = steerCount;
	}
	if (steerCount > max_vstupni_sig_servo || max_vstupni_sig_servo > MAXIMUM_VYSTUPNIHO_SIGNALU)
	{
		max_vstupni_sig_servo = steerCount;
	}
	attachInterrupt(STEERING_INPUT_PIN, SteerUpOccured, RISING);
	steerUp = 1;
}

void ThrotleUpOccured()
{
	attachInterrupt(THROTTLE_INPUT_PIN, ThrotleDownOccured, FALLING);
	throtleUp =0;
	throtleCountNew = micros();
}

void ThrotleDownOccured()
{
	attachInterrupt(THROTTLE_INPUT_PIN, ThrotleUpOccured, RISING);
	throtleCount = micros() - throtleCountNew;
	if (throtleCount != 0 && throtleCount < MAXIMUM_VYSTUPNIHO_SIGNALU && throttle_trimmed < TRIMM_DATA_LENGTH)
	{
		throttle_trim_data[throttle_trimmed] = throtleCount;
		throttle_trimmed++;
	}
	if (throtleCount >= THROTTLE_MIN_COUNT && (throtleCount < min_vstupni_sig_motor || min_vstupni_sig_motor == 0))
	{
		min_vstupni_sig_motor = throtleCount;
	}
	if (throtleCount <= THROTTLE_MAX_COUNT && (throtleCount > max_vstupni_sig_motor || max_vstupni_sig_motor > MAXIMUM_VYSTUPNIHO_SIGNALU))
	{
		max_vstupni_sig_motor = throtleCount;
	}
	throtleUp = 1;
}


char set_up_inputs()
{
	throtleUp = 0;
	steerUp = 0;
	throtleCount = 1500;
	steerCount = 1500;
	attachInterrupt(STEERING_INPUT_PIN, SteerUpOccured, RISING);
	attachInterrupt(THROTTLE_INPUT_PIN, ThrotleUpOccured, RISING);
	attachInterrupt(STEERING_INPUT_PIN, SteerDownOccured, FALLING);
	attachInterrupt(THROTTLE_INPUT_PIN, ThrotleDownOccured, FALLING);
	//register_filter();
}

char trimm_throttle()
{
	if (throttle_trimmed == TRIMM_DATA_LENGTH && stred_vstupniho_signalu_motor == 0)
	{
		int stred = 0;
		for (int i = 0; i < throttle_trimmed;i++)
		{
			stred += throttle_trim_data[i];
		}
		stred = stred / (throttle_trimmed);
		stred_vstupniho_signalu_motor = stred;
	}
}

char trimm_steering()
{
	if (servo_trimmed == TRIMM_DATA_LENGTH && stred_vstupniho_signalu_servo == 0)
	{
		int stred = 0;
		for (int i = 0; i < servo_trimmed;i++)
		{
			stred += servo_trim_data[i];
		}
		stred = stred / (servo_trimmed);
		stred_vstupniho_signalu_servo = stred;
	}
}

int get_throttle()
{
	trimm_throttle();
	if (throttle_trimmed < TRIMM_DATA_LENGTH)
		return 0;
	int vstup = throtleCount - stred_vstupniho_signalu_motor;
	int result = STRED_VYSTUPNIHO_SIGNALU_MOTOR;
	if (vstup > 0)
	{
		float scale = ((float)MAX_VYCHYLKA_AKCNIHO_ZASAHU)/(float)(max_vstupni_sig_motor - stred_vstupniho_signalu_motor);
		result = vstup*scale;
	}
	else if (vstup < 0)
	{
		float scale = ((float)MAX_VYCHYLKA_AKCNIHO_ZASAHU)/(float)(stred_vstupniho_signalu_motor - min_vstupni_sig_motor);
		result = vstup*scale;
	}
	else
		result = 0;
	
	return result;
}



char register_filter()
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 3;
	discrete->nominator_size = 3;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[2] = 0.006981;
	den[1] = - 0.1671;
	den[0] = 1;
	nom[0] = 0.21;
	nom[1] = 0.4199;
	nom[2] = 0.21;
	signal_block* low_pass = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete, low_pass);
	register_member(low_pass, &input_filter);
}

static int output_steering = 0;
static uint32_t time_micro = 0;

int get_steering()
{
	trimm_steering();
	if (servo_trimmed < TRIMM_DATA_LENGTH)
		return 0;
	int vstup = steerCount - stred_vstupniho_signalu_servo;
	int result = STRED_VYSTUPNIHO_SIGNALU_SERVO;
	if (vstup > 0)
	{
		float scale = ((float)MAX_VYCHYLKA_AKCNIHO_ZASAHU)/(float)(max_vstupni_sig_servo - stred_vstupniho_signalu_servo);
		result = vstup*scale;
	}
	else if (vstup < 0)
	{
		float scale = ((float)MAX_VYCHYLKA_AKCNIHO_ZASAHU)/(float)(stred_vstupniho_signalu_servo - min_vstupni_sig_servo);
		result = vstup*scale;
	}		
	else
		result = 0;
	//long tt = micros() - time_micro;
	//if (tt > SAMPLING_TIME_GYRO)
	//{
		//time_micro = micros();
		//output_steering = result - get_output(result, &input_filter);
	//}
	return result;
}

