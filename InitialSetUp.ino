
//
//
//




#include "Variables.h"
#include "sam.h"
#include "MPU_REGS.h"
#include "IMU.h"
#include "CurrentMeasurement.h"
#include "Telemetry.h"
#include "RPM_measurement.h"

char mpu_data_ready = 0;



void initialize_TWI() {
	
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
	g_APinDescription[PIN_WIRE_SDA].pPort,
	g_APinDescription[PIN_WIRE_SDA].ulPinType,
	g_APinDescription[PIN_WIRE_SDA].ulPin,
	g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
	g_APinDescription[PIN_WIRE_SCL].pPort,
	g_APinDescription[PIN_WIRE_SCL].ulPinType,
	g_APinDescription[PIN_WIRE_SCL].ulPin,
	g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
	NVIC_SetPriority(WIRE_ISR_ID, 1);
	NVIC_EnableIRQ(WIRE_ISR_ID);
	
			// Disable PDC channel
			WIRE_INTERFACE->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

		TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK);
		IMU_write_bit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
		IMU_write_bit(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
		IMU_write_bit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
		IMU_write_bit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,1 ,0);
		//IMU_write_bit(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_98);
	
}



char set_up_car()
{
	set_up_adc();
		initialize_TWI();
		//SerialUSB.begin(115200);
		set_up_telemetry();
		set_up_RPS();



		// put your setup code here, to run once:
		// PWM Set-up on pin: DAC1
		REG_PIOB_PER |= PIO_PER_P14 |PIO_PER_P21 | PIO_PER_P26;				//Enable PIO digitální piny 52 a 53 na Due
		REG_PIOB_ODR |= PIO_ODR_P21 | PIO_ODR_P14 |PIO_ODR_P26;				//Disable Output
		REG_PIOB_PUDR |= PIO_PUDR_P21 | PIO_PUDR_P14 |PIO_PUDR_P26;			//Disable PullUp
		

		
		
		REG_PMC_PCER1 |= PMC_PCER1_PID36;                     // Enable PWM
		REG_PIOC_ABSR |= PIO_ABSR_P24;                        // Set PWM pin perhipheral type A or B, in this case B
		REG_PIOC_PDR |= PIO_PDR_P24;                          // Set PWM pin to an output
		REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);     // Set the PWM clock rate to 2MHz (84MHz/42)
		REG_PWM_CMR7 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
		REG_PWM_CPRD7 = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz
		THROTTLE_REGISTER_LB = 1500;                                 // Set the PWM duty cycle to 1500 - centre the servo
		REG_PIOC_ABSR |= PIO_ABSR_P23;                        // Set PWM pin perhipheral type A or B, in this case B
		REG_PIOC_PDR |= PIO_PDR_P23;                          // Set PWM pin to an output
		REG_PWM_CMR6 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
		REG_PWM_CPRD6 = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz
		THROTTLE_REGISTER_RB = 1500;                                 // Set the PWM duty cycle to 1500 - centre the servo
		REG_PIOC_ABSR |= PIO_ABSR_P22;                        // Set PWM pin perhipheral type A or B, in this case B
		REG_PIOC_PDR |= PIO_PDR_P22;                          // Set PWM pin to an output
		REG_PWM_CMR5 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
		REG_PWM_CPRD5 = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz
		STEER_REGISTER = 1500;                                 // Set the PWM duty cycle to 1500 - centre the servo
		REG_PWM_ENA = PWM_ENA_CHID5 | PWM_ENA_CHID6 | PWM_ENA_CHID7;                          // Enable the PWM channel
	
}