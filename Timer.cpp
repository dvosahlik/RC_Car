// 
// 
// 

#include "Timer.h"
#include "Variables.h"
#include "Arduino.h"

void (*callback_loop)(void);

char restart_timer()
{
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
}

char set_up_timer(void (*loop)(void))
{
	callback_loop = loop;
	pmc_enable_periph_clk(ID_TC0);
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1;
	TC0->TC_CHANNEL[0].TC_RA = TIMER_COUNTER_VALUE;
	TC0->TC_CHANNEL[0].TC_IDR = ~(TC_IDR_CPAS);
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPAS;
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 5);
	NVIC_EnableIRQ(TC0_IRQn);
	restart_timer();
}





void TC0_Handler(void)
{
	REG_TC0_SR0;
	restart_timer();
	callback_loop();
}