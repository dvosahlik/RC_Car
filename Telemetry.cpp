// 
// 
// 

#include "Telemetry.h"
#include "Telemetrie.h"
#include "CurrentMeasurement.h"
#include "Arduino.h"
#include <string.h>
#include <avr/dtostrf.h>
#include "wiring.h"

#define BAUDRATE 115200

volatile debug_messages_t debug_messages;

double ENGINE_RPM_FEEDBACK_GAIN = 0.008;
double ENGINE_RPM_GAIN  = 25.55125;
communication_data_t com_data;

double const2 = -0.15;
double krat_rpm = 5;
double GYRO_ACCEL_FEEDBACK_GAIN = 0.001;
double ENGINE_FEEDBACK_GAIN_LIMITER = 0.00019;
double throttle_in = 0;
uint32_t throttle_start = 0;
uint16_t THROTTLE_DURATION = 1600;
uint32_t next_test = 0;	



char set_up_telemetry()
{
	pmc_enable_periph_clk(ID_USART3);
	USART3->US_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
	USART3->US_CR |= UART_CR_RSTSTA;
	USART3->US_MR = (3 << 6) | UART_MR_PAR_EVEN;
	USART3->US_BRGR = (SystemCoreClock / (BAUDRATE * 16));
	USART3->US_CR = UART_CR_TXEN | UART_CR_RXEN;
	NVIC_DisableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn, 20);
	NVIC_EnableIRQ(USART3_IRQn);
}

char set_up_adc_offset()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	int new_val = atoi(snumber);
	printf("Nova hodnota pro ADC OFFSET bude %d\n", new_val);
	MOTOR_HYSTERESIS_OFFSET_VOLTS = new_val;
}
char set_up_adc_offset_dur()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	int new_val = atoi(snumber);
	printf("Nova hodnota pro ADC OFFSET DURATION bude %d\n", new_val);
	MOTOR_HYSTERESIS_OFFSET_DURATION = new_val;
	
}

char set_up_throttle_in()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	int new_val = atoi(snumber);
	printf("Nova hodnota pro throttle_in bude %d\n", new_val);
	throttle_in = new_val;
	
}

char set_up_throttle_dur()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	int new_val = atoi(snumber);
	printf("Nova hodnota pro THROTTLE_DURATION bude %d\n", new_val);
	THROTTLE_DURATION = new_val;
	
}

char set_up_ENGINE_RPM_GAIN()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro ENGINE_RPM_FEEDBACK_GAIN bude nastavena\n");
	ENGINE_RPM_FEEDBACK_GAIN = new_val;
	
}

char set_up_const2()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro const2 bude nastavena\n");
	const2 = new_val;
	
}

char set_up_krat()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro krat_rpm bude nastavena\n");
	krat_rpm = new_val;
	
}

char set_up_accel_gyro()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro GYRO_ACCEL_FEEDBACK_GAIN bude nastavena\n");
	GYRO_ACCEL_FEEDBACK_GAIN = new_val;
	
}

char set_up_engine_fb_limiter()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro ENGINE_FEEDBACK_GAIN_LIMITER bude nastavena\n");
	ENGINE_FEEDBACK_GAIN_LIMITER = new_val;
	
}

char set_up_ENGINE_R_GAIN()
{
	char snumber[15];
	char* next = read_buffer;
	next++;
	memcpy(snumber, next, read_index - 1);
	next[read_index - 1] = '\0';
	double new_val = atof(snumber);
	printf("Nova hodnota pro ENGINE_RPM_FEEDBACK_GAIN bude nastavena\n");
	ENGINE_RPM_GAIN = new_val;
	
}

static char allow_print()
{
	char* next = read_buffer;
	char pp = next[read_index - 1] == '+';
	next++;
	switch (next[0])
	{
		case 'l':
			if (pp == 1)
				debug_messages.RPM_left = 1;
			else
				debug_messages.RPM_left = 0;
		break;
		case 'r':
			if (pp == 1)
				debug_messages.RPM_right = 1;
			else
				debug_messages.RPM_right = 0;
		break;
		case 't':
			if (pp == 1)
				debug_messages.loop_time = 1;
			else
				debug_messages.loop_time = 0;
		break;
		case 'd':
			if (pp == 1)
				debug_messages.telemtry_data = 1;
			else
				debug_messages.telemtry_data = 0;
		break;
		case 'v':
			if (pp == 1)
				debug_messages.vypocetni_cas = 1;
			else
				debug_messages.vypocetni_cas = 0;
		break;
		case 'a' :
			printf("pp je %d\n", pp);
		break;
	}
	if (pp)
	printf("Povoluji debugovaci zpravy typu %c\n", next[0]);
	else
	printf("Zakazuji debugovaci zpravy typu %c\n", next[0]);
}

void print_float(double val, char *string, char* out_string)
{
	char number[20];
	dtostrf(val, 8, 6, number);
	sprintf(out_string, string, number);
}

void print_telemetry_data(telemetry_data_t* data, int length)
{
	if (sending_tel)
		return;
	char result[128];
	int index = 0;
	for (int i = 0; i < length;i++)
	{
		char num[20];
		print_float(data[i].data, "%s", num);
		char* to_print = ",";
		if (i + 1 >= length)
			to_print = "\n";
		if (i == 0)
		{
			sprintf(result + index, "%lu,", data[i].time);
			int j = 0;
			while (result[index + j] != '\0')
			{
				j++;
			}
			index += j;
		}
		sprintf(result + index, "%s%s", num, to_print);
		int j = 0;
		while (result[index + j] != '\0')
		{
			j++;
		}
		index += j;
	}
	printf(result);
}




char check_buffer()
{
	if (zpracuj_buffer)
	{
		char task = read_buffer[0];
		switch (task)
		{
			case 'a' :
				set_up_adc_offset();
			break;
			case 'b' :
				set_up_throttle_in();
			break;
			case 'd' :
				set_up_adc_offset_dur();
			break;
			case 'D' :
				set_up_throttle_dur();
			break;
			case 'r' :
				set_up_ENGINE_R_GAIN();
			break;
			case 'f' :
				set_up_ENGINE_RPM_GAIN();
			break;
			case 'c':
				set_up_const2();
			break;
			case 'k':
				set_up_krat();
			break;
			case 'g':
				set_up_accel_gyro();
			break;
			case 'e':
				set_up_engine_fb_limiter();
			break;
			case 'n' :
				next_test++;
				printf("zmena jizdniho testu \n");
			break;
			case 't' :
			com_data.turn_off_rpm = !com_data.turn_off_rpm;
			printf("vypnuto nebo zapnuto t\n");
			break;
			case 'o' :
			com_data.turn_off_all = !com_data.turn_off_all;
			printf("vypnuto nebo zapnuto o\n");
			break;
			case 'p' :
				allow_print();
			break;
		}
		read_buffer[read_index] = '\0';
		read_index = 0;
		zpracuj_buffer = 0;
	}
}


