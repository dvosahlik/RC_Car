// 
// 
// 


#include "CurrentMeasurement.h"
#include "RingBuffer.h"
#include "signal_path_serial.h"
#include "Transfer_function.h"
#include <string.h>


#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif





uint8_t MOTOR_HYSTERESIS_OFFSET_VOLTS = 15;			//mV
uint8_t MOTOR_HYSTERESIS_OFFSET_DURATION = 4;
static volatile uint16_t *adc_buff;
static volatile uint16_t adc_buff_1[MOTOR_DATA_LENGTH];
static volatile uint16_t adc_buff_2[MOTOR_DATA_LENGTH];
static volatile uint16_t adc_buff_null[MOTOR_DATA_LENGTH];
static uint16_t left_motor_null = 0;
static uint16_t right_motor_null = 0;
static int32_t left_motor_mean = 0;
static int32_t right_motor_mean = 0;
static volatile char adc_initialized = 0;

static volatile char run_conversion = 1;


int test_adc = 0;
static uint32_t counter = 0;
static uint32_t counter_prevod = 0;

typedef enum {
	current,
	next_block,
	next_ok
	} adc_buff_use_t;
	
volatile adc_buff_use_t buffer_used = current;


/************************************************************************/
/* counts the mean value of the buffer                                                                     */
/************************************************************************/
static uint16_t get_mean(volatile uint16_t* buff, uint16_t length)
{
	uint32_t sum = 0;
	for(int i = 0;i < length;i++)
	{
		uint16_t val = buff[i];
		sum += val;
	}
	sum = sum/length;
	return sum;
}

/************************************************************************/
/* returns the mean value of absolut value of measured current in the motors                                                                     */
/************************************************************************/
static char get_mean_motors(volatile uint16_t* buff, uint16_t length, uint16_t zero_l, uint16_t zero_r, volatile int32_t* val_r, volatile int32_t* val_l)
{
	for (int j = 0; j < 2;j++)
	{
		int32_t sum = 0;
		int plus = j == 0 ? 0 : 1;
		uint16_t zero = j == 0 ? zero_l : zero_r;
		for(int i = 0;i < length/2;i++)
		{
			int16_t val = buff[i*2 + plus] - zero;
			sum += (val > 0 ? val : -1*val);
		}
		if (j == 0)
			*val_l = sum;
		else
			*val_r = sum;
	}
}

/************************************************************************/
/* ISR handler of ADC perifery                                                                     */
/************************************************************************/
void ADC_Handler(void)
{
	int f=ADC->ADC_ISR;
	

	 if (f & ADC_ISR_ENDRX){
		
		if (adc_initialized == 0 && counter_prevod > 100)
		{
			ADC->ADC_PTCR=0;
			memcpy((void*)adc_buff_null, (void*)adc_buff, sizeof(uint16_t)*MOTOR_DATA_LENGTH);
			ADC->ADC_CHDR=~((1<< CURRENT_LEFT_ADC_CHANELL) | (1 << CURRENT_RIGHT_ADC_CHANELL));   // disable all channels
			ADC->ADC_CHER=(1<< CURRENT_LEFT_ADC_CHANELL) | (1 << CURRENT_RIGHT_ADC_CHANELL);       //   use channels 0 and 2
			ADC->ADC_PTCR=1;
			adc_initialized = 1;
			
			return;
		}
		else if (counter_prevod <= 100)
		{
			counter_prevod++;
			return;
		}
		if (ADC->ADC_RNCR == 0)
		{
			ADC->ADC_RNPR=(uint32_t)NULL; // next receive pointer register DMA global_ADCounts_Arrayfer  points to second set of data
			// and "next count"
			ADC->ADC_RNCR=0;
			ADC->ADC_RPR = adc_buff == adc_buff_1 ? (uint32_t)adc_buff_1 : (uint32_t)adc_buff_2;
			adc_buff = adc_buff == adc_buff_1 ? adc_buff_2 : adc_buff_1;			
			ADC->ADC_RCR = MOTOR_DATA_LENGTH;
			
			buffer_used = current;
			
		}
		else
		{
			adc_buff = (uint16_t*)ADC->ADC_RPR;
			return;
		}
		 
		 test_adc = micros() - counter;
		counter = micros();
		
		
		}
}





/************************************************************************/
/* obsolete function for measuring the frequency based on the current signal. It was used to define from the measured frequency the RPM.                                                                     */
/************************************************************************/
static char measure_frequency(uint16_t* buff, int length, volatile int* frequency, int plus, uint16_t zero)
{
	uint32_t last_time = 0;		//in us
	uint32_t min_mcros = MIN_ALLOWED_PERIOD;// (float)1/(float)MAX_ALLOWED_FREQUENCY*(float)1000000;
	uint32_t min_mcros_half = min_mcros/(float)2;		//minimální odstup v us
	uint32_t crossed = 0;
	uint16_t measured_periods_length_max = 64;
	int measured_periods_length = 0;
	uint32_t measured_periods[measured_periods_length_max];
	char offset_reached = 0;
	uint32_t micros_offset = 0;
	for (int i = 0; i < length;i += 2)
	{
		if (micros_offset == 0 && offset_reached == 0 && (buff[i + plus] >= MOTOR_HYSTERESIS_OFFSET + zero || buff[i + plus] <= (zero - MOTOR_HYSTERESIS_OFFSET)))
		{
			micros_offset = i*ADC_SAMPLING_PERIOD_HALF;
		}
		else if (micros_offset != 0 && offset_reached == 0 && !(buff[i + plus] >= MOTOR_HYSTERESIS_OFFSET + zero || buff[i + plus] <= (zero - MOTOR_HYSTERESIS_OFFSET)))
		{
			micros_offset = 0;
		}
		else if (micros_offset != 0 && offset_reached == 0 && (i*ADC_SAMPLING_PERIOD_HALF - micros_offset) >= (MOTOR_HYSTERESIS_OFFSET_DURATION*ADC_SAMPLING_PERIOD))
		{
			offset_reached = 1;
		}
		if ((buff[i + plus] > zero && buff[i + plus - 2] <= zero) || (buff[i + plus] <= zero && buff[i + plus - 2] > zero))
		{
			if (!crossed)
			{
				if (last_time == 0)
				{
					last_time = i*ADC_SAMPLING_PERIOD_HALF;
				}
				else if (offset_reached == 1)
				{
					uint32_t curr_time = i*ADC_SAMPLING_PERIOD_HALF;
					if (min_mcros_half < (curr_time - last_time))
					{
						crossed = curr_time;
						offset_reached = 0;
						micros_offset = 0;
					}
				}
			}
			else
			{
				uint32_t curr_time = i*ADC_SAMPLING_PERIOD_HALF;
				if (min_mcros < (curr_time - last_time) && min_mcros_half < (curr_time - crossed) && offset_reached == 1)
				{
					
					if (measured_periods_length < measured_periods_length_max)
					{
						measured_periods[measured_periods_length] = curr_time - last_time;
						crossed = 0;
						last_time = curr_time;
						offset_reached = 0;
						micros_offset = 0;
						measured_periods_length++;
						if (measured_periods_length >= MOTOR_MAX_ITERATIONS)
							break;
					}
					else
						break;
				}
			}
		}
	}
	uint32_t mean = 0;
	for (int i = 0; i < measured_periods_length;i++)
	{
		mean += measured_periods[i]/measured_periods_length;
	}
	if (mean > 0 && mean > MIN_ALLOWED_PERIOD)
	{
		float deleno = mean;
		*frequency = 1/deleno*1000000;
	}
	else
		*frequency = 0;
}

serial_signal_path_t filter_current_l, filter_current_r;
/************************************************************************/
/* sets up the filter for the current measurements. This digital filter is used to suppers the noise generated by both the ADC and the current sensor.                                                                     */
/************************************************************************/
z_function* get_filter()
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 3;
	discrete->nominator_size = 3;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[2] = 0.9078;
	den[1] = - 1.906;
	den[0] = 1;
	nom[0] = 0.0005567;
	nom[1] = 0.001113;
	nom[2] = 0.0005567;
	return discrete;
} 
/************************************************************************/
/* Part of the filter used to suppress the measurement noise.                                                                */
/************************************************************************/
z_function* get_hp()
{
	z_function* discrete = (z_function*) malloc(sizeof(z_function));
	discrete->denominator_size = 3;
	discrete->nominator_size = 3;
	discrete->denominator = (double *)calloc(sizeof(double), discrete->denominator_size);
	discrete->nominator = (double *)calloc(sizeof(double), discrete->nominator_size);
	double* den = discrete->denominator;
	double* nom = discrete->nominator;
	den[2] = 0.9952;
	den[1] = - 1.995;
	den[0] = 1;
	nom[0] = 0.9976;
	nom[1] = - 1.995;
	nom[2] = 0.9976;
	return discrete;
}

/************************************************************************/
/* Registration of the digital filter used for ADC                                                                     */
/************************************************************************/
char register_low_pass_current()
{
	z_function* discrete_l = get_filter();
	z_function* discrete_r = get_filter();
	signal_block* low_pass_l = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete_l, low_pass_l);
	register_member(low_pass_l, &filter_current_l);
	signal_block* low_pass_r = (signal_block*)malloc(sizeof(signal_block));
	Transfer_function(discrete_r, low_pass_r);
	register_member(low_pass_r, &filter_current_r);
	
}


int cnts = 0;
#define MINUS_CONST_L 27300
#define MINUS_CONST_R 25000


/************************************************************************/
/* returns the measured values from current sensors, which are filtered by the digital filter.  
The values are absolute values of the currents.                                                                    */
/************************************************************************/
char get_motor_currents(volatile int32_t* value_l, volatile int32_t* value_r)
{
	*value_l = *value_r = test_adc;
	
	if (adc_initialized == 0)
	{
		buffer_used = current;
		return 1;
	}
	if (adc_initialized == 1)
	{
		adc_initialized = 2;
		uint16_t left_zero_buff[MOTOR_DATA_LENGTH/4];
		for (int i = 0; i < MOTOR_DATA_LENGTH/4;i++)
		{
			left_zero_buff[i] = adc_buff_null[i*4 + 1];
		}
		uint16_t right_zero_buff[MOTOR_DATA_LENGTH/4];
		for (int i = 0; i < MOTOR_DATA_LENGTH/4;i++)
		{
			right_zero_buff[i] = adc_buff_null[i*4 + 2];
		}
		left_motor_null = get_mean(left_zero_buff, MOTOR_DATA_LENGTH/4);
		right_motor_null = get_mean(right_zero_buff, MOTOR_DATA_LENGTH/4);
		buffer_used = current;
		return 1;
	}
	int32_t val_r, val_l;
	get_mean_motors(adc_buff, MOTOR_DATA_LENGTH, left_motor_null, right_motor_null, &val_r, &val_l);
	*value_l = get_output(val_l, &filter_current_l);
	*value_r = get_output(val_r, &filter_current_r);
	return 1;
}

/************************************************************************/
/* initialization method of the ADC periphery.                                                                     */
/************************************************************************/
char set_up_adc()
{


pmc_enable_periph_clk(ID_ADC);   //power management controller told to turn on adc
ADC->ADC_CR =1; //reset the adc
uint8_t prescal = 0xe;
ADC->ADC_MR= (prescal << 8) | ADC_MR_STARTUP_SUT512 | ADC_MR_SETTLING_AST5 ;//0x80182500 ;  //nastavení
// prescale :  ADC clock is mck/((prescale+1)*2).  mck is 84MHZ.
// prescale : 0x25=37
//ADC->ADC_EMR |= (1<<24);    // turn on channel numbers
ADC->ADC_CHDR=0xFFF0;   // disable all channels
REG_PIOA_PDR |= PIO_PDR_P2 | PIO_PDR_P3 |PIO_PDR_P4 |PIO_PDR_P6; 
REG_PIOA_PUDR |= PIO_PUDR_P2 | PIO_PUDR_P3 |PIO_PUDR_P4 |PIO_PUDR_P6; 
ADC->ADC_CHER=0x000f;       //   use channels 0,1,2 a 3

register_low_pass_current();
	NVIC_DisableIRQ(ADC_IRQn);
	NVIC_ClearPendingIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 20);
NVIC_EnableIRQ(ADC_IRQn); // interrupt controller set to enable adc.
ADC->ADC_IDR= ~(ADC_IDR_ENDRX | ADC_IDR_RXBUFF); // interrupt disable register, disables all interrupts but ENDRX
ADC->ADC_IER= ADC_IER_ENDRX | ADC_IER_RXBUFF;   // interrupt enable register, enables only ENDRX
delay(500);
// following are the DMA controller registers for this peripheral
// "receive buffer address"
ADC->ADC_RPR=(uint32_t)adc_buff_1;   // DMA receive pointer register  points to beginning of global_ADCount
// "receive count"
ADC->ADC_RCR=MOTOR_DATA_LENGTH;  //  receive counter set to 4
// "next-buffer address"
ADC->ADC_RNPR=(uint32_t)NULL; // next receive pointer register DMA global_ADCounts_Arrayfer  points to second set of data
// and "next count"
ADC->ADC_RNCR=0;   //  and next counter is set to 4
// "transmit control register"
ADC->ADC_PTCR= ADC_PTCR_RXTEN;  // transfer control register for the DMA is set to enable receiver channel requests
// now that all things are set up, it is safe to start the ADC.....
ADC->ADC_MR |= ADC_MR_FREERUN; // mode register of adc bit seven, free run, set to free running. starts ADC
	return 0;
}

