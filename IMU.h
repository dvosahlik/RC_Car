// IMU.h

#ifndef _IMU_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "signal_path_serial.h"
char get_accelerometer(volatile int16_t* y);
char get_gyro(volatile int16_t* z);

char IMU_get_velocity(volatile double *velocity, int16_t accel);

char get_accelerometer_x(volatile int16_t* x);

void onTxCmplt(void);

void onRxCmplt(void);

char IMU_write(uint8_t reg, uint8_t *data, uint16_t count);

char IMU_write_blocking(uint8_t reg, uint8_t *data, uint16_t count);

char IMU_write_bit(uint8_t reg, uint8_t bit, uint8_t length, uint8_t value);

char IMU_read_reg(uint8_t addr, uint8_t *data, uint16_t count);

char imu_working();

char imu_calibrate();

char IMU_read_next();

char IMU_filter(serial_signal_path_t* path);

extern volatile char twi_working;


#endif

