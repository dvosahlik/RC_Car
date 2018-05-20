#include "jizdni_testy.h"
#include "Timer.h"
#include "RPM_measurement.h"
#include <stdio.h>
#include "Telemetry.h"
#include "sysc.h"
#include "MPU_REGS.h"
#include "Transfer_function.h"
#include "IMU.h"
#include "signal_path_serial.h"
#include "Gain.h"
#include "RingBuffer.h"
#include "CurrentMeasurement.h"
#include "OutputMapper.h"
#include "InputPreprocessor.h"
#include "Variables.h"
#include "Telemetrie.h"




#include "Types.h"



#define GYRO_FEEDBACK_GAIN 0.013
#define ENGINE_FEEDFORWARD_GAIN 0.1
#define ENGINE_FEEDBACK_GAIN 0.0019


#define PRINT_INTERVAL 70		//ms
#define PRINT_INTERVAL_FAST 30		//ms

int cycle = 2000;
int pressed = 0;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;





void RPM_down();

serial_signal_path_t gyro_fb; 
serial_signal_path_t engine_fb;
serial_signal_path_t engine_rpm;
serial_signal_path_t engine_fb_limiter; 
serial_signal_path_t engine_ff; 
serial_signal_path_t filter_accel; 

serial_signal_path_t filter_RPS_l, filter_RPS_r;

z_function* get_filter_RPS()
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 3;
	discrete->nominator_size = 3;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[2] = 0.865;
	den[1] = - 1.86;
	den[0] = 1;
	nom[0] = 0.001224;
	nom[1] = 0.002447;
	nom[2] = 0.001224;
	return discrete;
}

char register_low_pass_RPS()
{
	z_function* discrete_l = get_filter_RPS();
	z_function* discrete_r = get_filter_RPS();
	signal_block* low_pass_l = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete_l, low_pass_l);
	register_member(low_pass_l, &filter_RPS_l);
	signal_block* low_pass_r = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete_r, low_pass_r);
	register_member(low_pass_r, &filter_RPS_r);
}





void main_loop();

char engine_feedforward_path_set_up()
{
	engine_ff.blocks = NULL;
	engine_ff.size_used = 0;
	engine_ff.size = 0;
	
	signal_block* i = (signal_block*)malloc(sizeof(signal_block));
	gain(ENGINE_FEEDFORWARD_GAIN, i);
	register_member(i, &engine_ff);
	

}

char engine_feedback_path_set_up()
{
	engine_fb.blocks = NULL;
	engine_fb.size_used = 0;
	engine_fb.size = 0;
	
	signal_block* i = (signal_block*)malloc(sizeof(signal_block));
	gain(ENGINE_FEEDBACK_GAIN, i);
	register_member(i, &engine_fb);
	

}

char engine_rpm_feedback_path_set_up()
{
	engine_rpm.blocks = NULL;
	engine_rpm.size_used = 0;
	engine_rpm.size = 0;
	
	signal_block* i = (signal_block*)malloc(sizeof(signal_block));
	var_gain(&ENGINE_RPM_FEEDBACK_GAIN, i);
	register_member(i, &engine_rpm);
	

}


char engine_feedback_limiter_path_set_up()
{
	engine_fb_limiter.blocks = NULL;
	engine_fb_limiter.size_used = 0;
	engine_fb_limiter.size = 0;
	
	signal_block* i = (signal_block*)malloc(sizeof(signal_block));
	var_gain(&ENGINE_FEEDBACK_GAIN_LIMITER, i);
	register_member(i, &engine_fb_limiter);
	

}


char register_low_pass()
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
	register_member(low_pass, &gyro_fb);
}



char gyro_feedback_path_set_up()
{
	gyro_fb.blocks = NULL;
	gyro_fb.size_used = 0;
	gyro_fb.size = 0;
	register_low_pass();
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 2;
	discrete->nominator_size = 2;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[1] = - 0.9761;
	den[0] = 1;
	nom[0] =   0.9881;
	nom[1] = - 0.9881;
	signal_block* high_pass = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete, high_pass);
	//register_member(high_pass, &gyro_fb);
	
	signal_block* i = (signal_block*)malloc(sizeof(signal_block));
	gain(GYRO_FEEDBACK_GAIN, i);
	register_member(i, &gyro_fb);
	

}


/************************************************************************/
/* initialization sequence used to configure all the signal paths and peripheries.                                                                     */
/************************************************************************/

void setup(){
	
set_up_car();
register_low_pass_RPS();
gyro_feedback_path_set_up();
engine_feedforward_path_set_up();
engine_feedback_path_set_up();
engine_feedback_limiter_path_set_up();
IMU_filter(&filter_accel);
	set_up_inputs();
	scale_throttle_output(100, 98);
	set_up_timer(main_loop);
	
}




uint8_t IMU_data[6];
int count = 0;
int setted_up = 0;


/************************************************************************/
/* used for printing the telemetry data if enabled.                                                                     */
/************************************************************************/
char print_tel(double RPM_L, double RPM_R, double velocity, double akcelerometr, double current_left, double current_right, double throttle, double steer, double gyro)
{
	telemetry_data_t tel[9];
	tel[0].data = RPM_L;
	tel[0].identifier = 'l';
	tel[1].data = RPM_R;
	tel[1].identifier = 'r';
	tel[2].data = velocity;
	tel[2].identifier = 'v';
	tel[3].data = akcelerometr;
	tel[3].identifier = 'a';
	tel[4].data = current_left;
	tel[4].identifier = 'b';
	tel[5].data = current_right;
	tel[5].identifier = 'c';
	tel[6].data = throttle;
	tel[6].identifier = 'd';
	tel[7].data = steer;
	tel[7].identifier = 'e';
	tel[8].data = gyro;
	tel[8].identifier = 'f';
	tel[0].time = micros();
	print_telemetry_data(tel, 9);
}

float max_current = 0;
int output_steering = 0;
static uint32_t time_micro = 0;
static uint32_t vypocetni_cas = 0;
uint32_t time_micro2 = 0;
int conversion = 0;
int motor_left = 0;
int motor_right = 0;
static double difference = 0;
uint32_t interval = 0;
uint32_t interval_fast = 0;
static volatile double RPM_left = 0;
static volatile double RPM_right = 0;
static volatile int32_t current_left_stred = 0;
static volatile int32_t current_right_stred = 0;
static volatile int32_t current_left = 0;
static volatile int32_t current_right = 0;
static volatile int16_t gyro_x, gyro_y, gyro_z;
static volatile int16_t accel_x, accel_y, accel_z;
static volatile double vel = 0;
static inputs_t inputs;
static uint32_t throttle_time = 0;
static int throttle_old = 0;


/************************************************************************/
/* The main loop. Used for time unconstrained algorithms such as printing the telemetry                                                                     */
/************************************************************************/
void loop(){
	char changed = 0;
	unsigned int nextVal = 0;
	//printf("test");
	if (!count)
	{
		SerialUSB.println("Startuji");
		//set_up_interrupt();
		//setted_up = test_mpu_connection();
		SerialUSB.println();
		SerialUSB.print("mode register ="); SerialUSB.println(REG_ADC_MR, HEX);
		SerialUSB.print("channel enabled register ="); SerialUSB.println(REG_ADC_CHSR, HEX);
		SerialUSB.print("sequence register1 ="); SerialUSB.println(REG_ADC_SEQR1, HEX);
		SerialUSB.print("interrupts ="); SerialUSB.println(REG_ADC_IMR, HEX);
		printf("starting\n\r");
		imu_calibrate();
		printf("imu_calibarted\n\r");
		
	}
	count = 1;
	
	IMU_read_next();
	
	
	char p_allowed = micros() - interval >= PRINT_INTERVAL*1000;
	char p_allowed_fast = micros() - interval_fast >= PRINT_INTERVAL_FAST*1000;
	if (p_allowed)
	interval = micros();
	if (p_allowed_fast)
	interval_fast = micros();
	check_buffer();
	if (debug_messages.loop_time && p_allowed)
			printf("smyckovy cas je %u\n", time_micro2);
	if (debug_messages.RPM_left == 1  && p_allowed && RPM_left != 0)
	{
		printf("otacky vlevo %d\n", RPM_left);
	}
	if (debug_messages.RPM_right  && p_allowed && RPM_right != 0)
	{
		printf("otacky vpravo %d\n", RPM_right);
	}
	if ((debug_messages.telemtry_data && p_allowed) || (remaining_test_time() && p_allowed_fast))
	{
		print_tel(RPM_left, RPM_right, vel, accel_y, current_left, current_right, inputs.throttle, inputs.steering, gyro_z);
	}
	if (debug_messages.vypocetni_cas && p_allowed)
	{
		printf("vypocetni cas je %u\n", vypocetni_cas);
	}
}




/************************************************************************/
/* main regulation loop used for time constrained algorithms.                                                                     */
/************************************************************************/
void main_loop()
{
	uint32_t cas = micros();
	inputs.throttle = get_throttle();
	inputs.steering = get_steering();
	test_inputs(&inputs);
	get_gyro(&gyro_z);
	get_accelerometer_x(&accel_x);
	vel = 0;
				int32_t rps_r = get_RPS_right();
				int32_t rps_l = get_RPS_left();
				int32_t diff_r = RPM_right - rps_r;
				diff_r = get_output(diff_r , &filter_RPS_r);
				int32_t diff_l = get_output((RPM_left - rps_l), &filter_RPS_l);
				diff_l = diff_l > 0 ? diff_l : -1*diff_l;
				diff_r = diff_r > 0 ? diff_r: -1*diff_r;
				RPM_right = diff_r;
				RPM_left = rps_l;
				get_motor_currents(&current_right, &current_left);
				if (inputs.throttle < 30 && inputs.throttle > -70)
				{
					current_left_stred = current_left;
					current_right_stred = current_right;
				}
				current_right -= current_right_stred;
				current_left -= current_left_stred;
				current_right = abs(current_right);
				current_left = abs(current_left);
	get_accelerometer(&accel_y);
		IMU_get_velocity(&vel, accel_y);
	if (com_data.turn_off_all || inputs.throttle < -100)
	{
		torque_vector torque;
		torque.left_wheel = inputs.throttle;
		torque.right_wheel = inputs.throttle;
		map_to_throttle(&torque);
		map_to_steering(inputs.steering);
		return;
	}
	
	accel_y = get_output(accel_y, &filter_accel);

	time_micro2 = micros() - time_micro;
	time_micro = micros();
	output_steering = inputs.steering - get_output(gyro_z, &gyro_fb);
	double st_ff = get_output(inputs.steering, &engine_ff);
	double g_fb = get_output(gyro_z, &engine_fb);	
	double minus_stranove = 0;// Abs(GYRO_ACCEL_FEEDBACK_GAIN*(accel_x - gyro_z));
	double plus = - st_ff + g_fb - get_output(gyro_z, &engine_fb_limiter) - minus_stranove;
	int test;
	motor_left = inputs.throttle + plus;
	plus = st_ff - g_fb - get_output(gyro_z, &engine_fb_limiter) - minus_stranove;
	motor_right = inputs.throttle + plus;
	
	if (!com_data.turn_off_rpm)
	{	

		double krat = 1;
		if (inputs.throttle > 30 && RPM_left > 200 && RPM_right > 200)
		{
			difference =  max((diff_l*krat - current_left)*const2, 0);
			difference = (difference + max((diff_r*krat - current_right)*const2, 0))/2;
			
			motor_left -= difference;
			motor_right -= difference;			
		}
		else
			difference = 0;
		difference = diff_l;

	}
	if (inputs.throttle >= -50 && motor_right < 0)
	motor_right = 10;
	if (inputs.throttle >= -50 && motor_left < 0)
	motor_left = 10;
	torque_vector torque;
	torque.left_wheel = motor_left;
	torque.right_wheel = motor_right;
	map_to_throttle(&torque);
	map_to_steering(output_steering);
	vypocetni_cas = micros() - cas;
}
