#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "Timer.h"

void Timer::initilize_Timer(void){
	//11.4 enable the clock for timer
	SYSCTL_RCGCTIMER_R |= 0b00000001;
	//1)GPTM control- Timer0 Subtimer A is initially cleared. Can also select timer A or B or select what timer will do at each mode.
	TIMER0_CTL_R &= ~(1<<0);
	//2)GPTM configuratation register
	TIMER0_CFG_R |= 0x00000000;
	//3)Configure the Timer n mode register (one shot or periodic pg.722 of Data sheet). We selected 0x02 for periodic mode
	TIMER0_TAMR_R |= (0x1<<0);
	//4) The direction of counter (count up)
	TIMER0_TAMR_R |= (1<<4);
	//7)Enable the timer A
	TIMER0_CTL_R |= (1<<0);
}


