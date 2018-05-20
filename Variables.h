/*
 * Variables.h
 *
 * Created: 16.02.2018 18:33:24
 *  Author: David
 */ 


#ifndef VARIABLES_H_
#define VARIABLES_H_

#define STEER_REGISTER REG_PWM_CDTY5
#define THROTTLE_REGISTER_LB REG_PWM_CDTY7
#define THROTTLE_REGISTER_RB REG_PWM_CDTY6
#define BUTTON_PIN 41
#define RIGHT_RPM 26
#define MPU_ADDRR 0x68	// I2C address of the MPU-6050
#define STEERING_INPUT_PIN	53
#define THROTTLE_INPUT_PIN 22

#define SAMPLING_TIME_GYRO  3846		//us
#define TIMER_COUNTER_VALUE                     SAMPLING_TIME_GYRO*(84/2)      //počet iterací při MCK = 84 MHz a použití TIMER_CLOCK1 


extern char mpu_data_ready;


#endif /* VARIABLES_H_ */