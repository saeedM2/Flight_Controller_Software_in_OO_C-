#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "Actuators.h"
#include "Communication.h"

void Actuators::PWM_initilization(void){
	uint16_t period=400;
	//PWM INITILIZATION
	SYSCTL_RCGCPWM_R |= 0b00000011;;//unlock pwm
	SYSCTL_RCGC2_R|=0b00011111;//choose the port that recieves clock
	while((SYSCTL_PRGPIO_R & 0x00000008) == 0); // wait until port D is ready
	while((SYSCTL_PRGPIO_R & 0x00000010) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000004) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000002) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000001) == 0);

	GPIO_PORTB_AFSEL_R |=0b01010000;//
	GPIO_PORTA_AFSEL_R |=0b01000000;//
	GPIO_PORTE_AFSEL_R |=0b00010000;//

	GPIO_PORTB_PCTL_R &= ~0xFF0F0000;
	GPIO_PORTB_PCTL_R |=0x04040000;//

	GPIO_PORTA_PCTL_R &= ~0x0F000000;
	GPIO_PORTA_PCTL_R |=  0x05000000;//

	GPIO_PORTE_PCTL_R &= ~0x000F0000;
	GPIO_PORTE_PCTL_R |=0x00040000;//

	GPIO_PORTB_AMSEL_R &=~(0b01010000); //
	GPIO_PORTB_DEN_R |=0b01010000;   //

	GPIO_PORTA_AMSEL_R &= ~(0b01000000); //
	GPIO_PORTA_DEN_R   |=   0b01000000;   //

	GPIO_PORTE_AMSEL_R &= ~(0b00010000);//
	GPIO_PORTE_DEN_R |= 0b00010000; //

	SYSCTL_RCC_R |=1<<20; //The PWM clock divider is the source for the PWM clock.
	SYSCTL_RCC_R |=SYSCTL_RCC_PWMDIV_M;//clear the control register
	SYSCTL_RCC_R |=SYSCTL_RCC_PWMDIV_8; //configure for /8 divider

	PWM0_0_CTL_R = 0b00000000;////B6
	PWM0_1_CTL_R = 0b00000000;//B4
	PWM0_2_CTL_R = 0b00000000;//E4
	PWM1_1_CTL_R = 0b00000000;//A6

	PWM0_0_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; //
	PWM0_1_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; //
	PWM0_2_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO;//
	PWM1_1_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; // CMPA down, if counter=CMPA drive PWMA low, high if counter=LOAD
	//PWM0_3_GENB_R |= 0x0000080C; // CMPB down, if counter=CMPB drive PWMB low, high if counter=LOAD

	PWM0_0_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM0_1_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM0_2_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM1_1_CTL_R &= ~0x00000002; // count down mode (cleared second bit)

	PWM0_0_LOAD_R |=period-1; //
	PWM0_1_LOAD_R |=period-1; //
	PWM0_2_LOAD_R |=period-1;//
	PWM1_1_LOAD_R |=period-1; //

	PWM0_0_CTL_R |= (PWM_0_CTL_ENABLE);//
	PWM0_1_CTL_R |= (PWM_1_CTL_ENABLE);//
	PWM0_2_CTL_R |= (PWM_2_CTL_ENABLE);//
	PWM1_1_CTL_R |= (PWM_1_CTL_ENABLE);//

	//Here you enable the PWM from Generator 0 1 2 and 3 into (each generator has two channels) to a specigic pin
	PWM0_ENABLE_R |=0b00010101;//
	PWM1_ENABLE_R |=0b00000100;//
}

void Actuators::PWM_adjustment(float pid_pitch, float pid_roll, float pid_yaw,uint16_t baseSpeed){

	//Instanciate object
	communication_setup Communicate;

	//initilize var
	int16_t sum1=0;
	int16_t sum2=0;
	int16_t sum3=0;
	int16_t sum4=0;

	//prior to feeding the PWM registers calculaate the overall sum and check if it breaks any speed bounds
	//Yaw adjustment Should be added later or else the Quadcopter will spin!!
	sum1 = baseSpeed - pid_pitch + pid_roll + pid_yaw;
	sum2 = baseSpeed + pid_pitch + pid_roll - pid_yaw;
	sum3 = baseSpeed - pid_pitch - pid_roll - pid_yaw;
	sum4 = baseSpeed + pid_pitch - pid_roll + pid_yaw;

	//Pitch Upper Bound
	if(sum2 > (250) && sum4 > (250))
	{
		sum2=250;
		sum4=250;
	}
	else if(sum1 > (250) && sum3 > (250))
	{
		sum1=250;
		sum3=250;
	}
	//Pitch Lower Bound
	if (sum2 < (34) && sum4 < (34))
	{
		sum2=35;
		sum4=35;
	}
	else if(sum1 < (34) && sum3 < (34))
	{
		sum1=35;
		sum3=35;
	}
	//Roll Upper Bound
	if(sum1 > (250) && sum2 > (250))
		{
			sum1=250;
			sum2=250;
		}
	else if(sum3 > (250) && sum4 > (250))
	{
		sum3=250;
		sum4=250;
	}
	//Roll Lower Bound
	if (sum1 < (34) && sum2 < (34))
	{
		sum1=35;
		sum2=35;
	}
	else if(sum3 < (34) && sum4 < (34))
	{
		sum3=35;
		sum4=35;
	}
	//YAW Upper Bound
	if(sum1 > (250) && sum4 > (250))
	{
		sum1=250;
		sum4=250;
	}
	else if(sum2 > (250) && sum3 > (250))
	{
		sum2=250;
		sum3=250;
	}
	//YAW Lower Bound
	if (sum1 < (34) && sum4 < (34))
	{
		sum1=35;
		sum4=35;
	}
	else if(sum2 < (34) && sum3 < (34))
	{
		sum2=35;
		sum3=35;
	}
	Communicate.printString("\t");
	Communicate.print(sum1);//pitch
	Communicate.printString("\t");
	Communicate.print(sum2);//pitch
	Communicate.printString("\t");
	Communicate.print(sum3);//pitch
	Communicate.printString("\t");
	Communicate.print(sum4);//pitch
	Communicate.printString("\t");

	//Change the speeds here:
	PWM1_1_CMPA_R =sum4;//(green)//motor4   //ccw rotor
	PWM0_0_CMPA_R =sum2;//(Yellow)//motor2  //cw rotor
	PWM0_2_CMPA_R =sum1;//(RED)//motor1     //ccw rotor
	PWM0_1_CMPA_R =sum3;//(ORANGE)//motor3  //cw rotor
}
