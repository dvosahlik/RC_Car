// 
// 
// 

#include "RPM_measurement.h"
#include "CurrentMeasurement.h"
#include "Transfer_function.h"
#include "signal_path_serial.h"

#include <stdlib.h>
#define RPM_LEFT_PIN	26
#define RPM_RIGHT_PIN	27


void rpm_left_up();
void rpm_right_up();

	


uint32_t left_count, left_count_new, l_pom_up;
uint32_t right_count, right_count_new, r_pom_up;

/************************************************************************/
/* ISR for falling edge event on left motor RPM measurement pin                                                                     */
/************************************************************************/
void rpm_left_down()
{
	if (micros() - l_pom_up >= MIN_ALLOWED_PERIOD/2 && micros() - left_count_new >= MIN_ALLOWED_PERIOD)
	{
		left_count = micros() - left_count_new;
		left_count_new = micros();
	}
}

/************************************************************************/
/* ISR for rising edge event on left motor RPM measurement                                                                     */
/************************************************************************/
void rpm_left_up()
{
	if (micros() - left_count_new >= MIN_ALLOWED_PERIOD/2 && micros() - l_pom_up >= MIN_ALLOWED_PERIOD)
	{
		l_pom_up = micros();
	}
}

/************************************************************************/
/* ISR for falling edge event on right motor RPM measurement                                                                     */
/************************************************************************/
void rpm_right_down()
{
	if (micros() - r_pom_up >= MIN_ALLOWED_PERIOD/2 && micros() - right_count_new >= MIN_ALLOWED_PERIOD)
	{
		right_count = micros() - right_count_new;
		right_count_new = micros();
	}
}

/************************************************************************/
/* ISR for rising edge event on right motor RPM measurement                                                                     */
/************************************************************************/
void rpm_right_up()
{
	if (micros() - right_count_new >= MIN_ALLOWED_PERIOD/2 && micros() - r_pom_up >= MIN_ALLOWED_PERIOD)
	{
		r_pom_up = micros();
	}
}



int byl = 0;
/************************************************************************/
/* returns the RPS (Revolution per second) for the left motor                                                                     */
/************************************************************************/
double get_RPS_left()			//rotation per second
{
	double result = (double)1/left_count*(double)1000000;
	if (result < MIN_ALLOWED_FREQUENCY)
		return 0;
	else
		return result;
}

/************************************************************************/
/* returns the RPS (Revolution per second) for the right motor                                                                        */
/************************************************************************/
double get_RPS_right()			//rotation per second
{
	double result = (double)1/right_count*(double)1000000;
	if (result < MIN_ALLOWED_FREQUENCY)
		return 0;
	else
		return result;
}

/************************************************************************/
/* Initial sequence for setting up the RPM measurements                                                                     */
/************************************************************************/
char set_up_RPS()
{
			REG_PIOD_PER |= PIO_PER_P1 |PIO_PER_P2 ;				//Enable PIO digitální piny 52 a 53 na Due
			REG_PIOD_ODR |= PIO_ODR_P1 | PIO_ODR_P2 ;				//Disable Output
			REG_PIOD_PUDR |= PIO_PUDR_P1 | PIO_PUDR_P2 ;			//Disable PullUp
			REG_PIOD_IER |= PIO_IER_P1 | PIO_IER_P2;
			
			NVIC_DisableIRQ(PIOD_IRQn);
			NVIC_ClearPendingIRQ(PIOD_IRQn);
			NVIC_SetPriority(PIOD_IRQn, 17);
			NVIC_EnableIRQ(PIOD_IRQn);
			
}

/************************************************************************/
/* ISR for PIO events                                                                     */
/************************************************************************/
void PIOD_Handler(void)
{
	uint32_t isr = PIOD->PIO_ISR;
	byl = 1;
	if ((isr & 2) == 2)
	{
		if (REG_PIOD_PDSR & 2)
			rpm_left_up();
		else
			rpm_left_down();
	}
	if (isr & 4)
	{
		if (REG_PIOD_PDSR & 4)
			rpm_right_up();
		else
			rpm_right_down();
	}
}
