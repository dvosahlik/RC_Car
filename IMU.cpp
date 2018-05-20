// 
// 
// 

#include "IMU.h"



#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Variables.h"
#include "MPU_REGS.h"
#include "limits.h"
#include "signal_path_serial.h"
#include "Transfer_function.h"

#define MAX_ACCEL_VALUE 19.62
#define CALLIBRATING_ITERATIONS 10000		
#define ACCEL_CONSTANT (double)MAX_ACCEL_VALUE/(double)1000000/(double)SHRT_MAX

static uint8_t buffer_out[6];



volatile char twi_working = 0;

bool didWeRead;
enum ServiceState : uint8_t {
	FinishedReading,
	FinishedWriting,
	FinishedWaiting,
};

char imu_working()
{
	return twi_working;
}

static inline void TWI_PDCWrite(uint8_t *data, uint16_t count) {
	WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
	WIRE_INTERFACE->TWI_TPR = (uint32_t)data;
	WIRE_INTERFACE->TWI_TCR = count;
	WIRE_INTERFACE->TWI_TNPR = 0;
	WIRE_INTERFACE->TWI_TNCR = 0;
}
static inline void TWI_PDCRead(uint8_t *data, uint16_t count) {
	WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
	WIRE_INTERFACE->TWI_RPR = (uint32_t)data;
	WIRE_INTERFACE->TWI_RCR = count - 1;
	WIRE_INTERFACE->TWI_RNPR = 0;
	WIRE_INTERFACE->TWI_RNCR = 0;
}

static inline void TWI_MasterModeWrite(uint8_t deviceAddress, uint8_t reg) {
	WIRE_INTERFACE->TWI_MMR = TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_DADR(deviceAddress);
	WIRE_INTERFACE->TWI_IADR = reg;
	WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
}
static inline void TWI_MasterModeRead(uint8_t deviceAddress, uint8_t reg) {
	WIRE_INTERFACE->TWI_MMR = TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_DADR(deviceAddress) | TWI_MMR_MREAD;
	WIRE_INTERFACE->TWI_IADR = reg;
	WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
}

static inline void TWI_Write() {
	WIRE_INTERFACE->TWI_IER = TWI_IER_ENDTX;	
	//WIRE_INTERFACE->TWI_CR = TWI_CR_START;
	WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTEN;
}
static inline void TWI_Read() {
	WIRE_INTERFACE->TWI_IER = TWI_IER_ENDRX;
	WIRE_INTERFACE->TWI_CR = TWI_CR_START;
	WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTEN;
}


char IMU_write(uint8_t reg, uint8_t *data, uint16_t count) {
	if (twi_working)
		return 0;
	twi_working = 1;
	TWI_PDCWrite(data, count);
	TWI_MasterModeWrite(MPU_ADDRR, reg);
	TWI_Write();
	return 1;
}

char IMU_write_byte(uint8_t reg, uint8_t value)
{
	WIRE_INTERFACE->TWI_MMR = TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_DADR(MPU_ADDRR);
	WIRE_INTERFACE->TWI_IADR = reg;
	WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
	WIRE_INTERFACE->TWI_THR = value;
	while ((WIRE_INTERFACE->TWI_SR & TWI_SR_TXRDY) != TWI_SR_TXRDY);
	WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
	while ((WIRE_INTERFACE->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP);
}

char IMU_write_byte16(uint8_t reg, int16_t value)
{
	IMU_write_byte(reg, ((value & 0xFF00) >> 8));
	IMU_write_byte((reg + 1), (value & 0xFF));
}

char IMU_write_bit(uint8_t reg, uint8_t bit, uint8_t length, uint8_t value) {
	uint8_t write;
	IMU_read_reg(reg, &write, 1);
	while(twi_working);
	uint8_t mask = ((1 << length) - 1) << (bit - length + 1);
	value <<= (bit - length + 1); // shift data into correct position
	value &= mask; // zero all non-important bits in data
	write &= ~(mask); // zero all important bits in existing byte
	write |= value; // combine data with existing byte
	IMU_write_byte(reg, write);
}

char IMU_write_blocking(uint8_t reg, uint8_t *data, uint16_t count) {
	while(!IMU_write(reg, data, count));
	while(twi_working)
	{
	}
}


uint8_t* read_data;
uint16_t read_data_count;
char IMU_read_reg(uint8_t addr, uint8_t *data, uint16_t count) {
	if (twi_working)
		return 0;
	twi_working = 1;
	read_data = data;
	read_data_count = count;
	TWI_PDCRead(data, count);
	TWI_MasterModeRead(MPU_ADDRR, addr);
	TWI_Read();
	return 1;
}

char read_index_IMU = 0;

char IMU_read_next()
{
	switch (read_index_IMU)
	{
		case 0:
			if (IMU_read_reg(MPU6050_RA_GYRO_ZOUT_H, buffer_out, 2))
				read_index_IMU = 1;
		break;
		case 1:
			if (IMU_read_reg(MPU6050_RA_ACCEL_XOUT_H, buffer_out + 2, 4))
				read_index_IMU = 0;
		break;
	}
}



void onTransmitCmpltCallback(void)
{
	
}

uint32_t last_time_acceleration = 0;

void onReceiveCmpltCallback(uint8_t last_value)
{
	read_data[read_data_count - 1] = last_value;
}


void TWI1_Handler() 
{
	uint32_t sr = WIRE_INTERFACE->TWI_SR;
	uint32_t cr = WIRE_INTERFACE->TWI_CR;
	WIRE_INTERFACE->TWI_IDR = TWI_IDR_ENDRX | TWI_IDR_ENDTX;
	WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;
	WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
	uint8_t last_value = 0;
	if ((sr & TWI_SR_ENDRX) == TWI_SR_ENDRX)
	{
		while (( WIRE_INTERFACE->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY)
		{
			
		}
		last_value = WIRE_INTERFACE->TWI_RHR;
	}
	while ((WIRE_INTERFACE->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP);
	if ((sr & TWI_SR_ENDRX) == TWI_SR_ENDRX)
	{
		if (onReceiveCmpltCallback)
		onReceiveCmpltCallback(last_value);
	}
	
	if ((sr & TWI_SR_ENDTX) == TWI_SR_ENDTX)
	{
		if (onTransmitCmpltCallback)
		onTransmitCmpltCallback();
	}
	twi_working = 0;
}

int16_t accel_y_offset = 0;
int16_t accel_x_offset = 0;
int16_t gyro_z_offset = 0;

char get_accelerometer_x(volatile int16_t* x)
{
	*x = (((int16_t)buffer_out[2]) << 8) | buffer_out[3];
	*x -= accel_x_offset;
}

char get_accelerometer(volatile int16_t* y)
{
	*y = (((int16_t)buffer_out[4]) << 8) | buffer_out[5];
	*y -= accel_y_offset;
}

double vel_old = 0;
int16_t accel_old = 0;
uint32_t micros_velocity = 0;

char register_low_pass(serial_signal_path_t* fb)
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 4;
	discrete->nominator_size = 4;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[3] = - 0.5589;
	den[2] =  2.036;
	den[1] = - 2.471;
	den[0] = 1;
	nom[0] = 0.0006847;
	nom[1] = 0.002054;
	nom[2] = 0.002054;
	nom[3] = 0.0006847;
	signal_block* low_pass = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete, low_pass);
	register_member(low_pass, fb);
}

char register_high_pass(serial_signal_path_t* fb)
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 3;
	discrete->nominator_size = 3;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[2] = 0.9528;
	den[1] = - 1.952;
	den[0] = 1;
	nom[0] =  0.9763;
	nom[1] = - 1.953;
	nom[2] = 0.9763;
	signal_block* low_pass = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete, low_pass);
	register_member(low_pass, fb);
}

char IMU_filter(serial_signal_path_t* path)
{
	register_low_pass(path);
	//register_high_pass(path);
}

char IMU_get_velocity(volatile double *velocity, int16_t accel)
{
	*velocity = 0;
	//if (accel < 200 && accel > -200)
		//accel = 0;
	if (micros_velocity == 0)
	{
		micros_velocity = micros();
		return 0;
	}
	micros_velocity = micros() - micros_velocity;
	*velocity = vel_old + ((accel+accel_old)/(double)2*micros_velocity*ACCEL_CONSTANT);
	vel_old = *velocity;
	accel_old = accel;
	micros_velocity = micros();
	return 1;
}

char get_gyro(volatile int16_t* z)
{
	*z = (((int16_t)buffer_out[0]) << 8) | buffer_out[1];
	*z -= gyro_z_offset;
}


char imu_calibrate()
{
	uint8_t cal_buff[14];
	int i = 0;
	int mean_accel_y = 0;
	int mean_accel_x = 0;
	int mean_gyro = 0;
	while(i < CALLIBRATING_ITERATIONS)
	{
		int16_t accel_y, accel_x, gyro;
		IMU_read_next();
		while (twi_working);
		IMU_read_next();
		while (twi_working);
		IMU_read_next();
		while (twi_working);
		get_accelerometer(&accel_y);
		get_accelerometer_x(&accel_x);
		get_gyro(&gyro);
		mean_accel_y += accel_y;
		mean_accel_x += accel_x;
		mean_gyro += gyro;
		i++;
	}
	mean_accel_y = mean_accel_y/(float)i;
	accel_y_offset = mean_accel_y;
	
	mean_accel_x = mean_accel_x/(float)i;
	accel_x_offset = mean_accel_x;
	
	mean_gyro = mean_gyro/(float)i;
	gyro_z_offset = mean_gyro;
	return 1;
}

