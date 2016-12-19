#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "ControlSystem.h"
#include "Communication.h"

#define iMax 250
#define iMin -250

#define pitch_kp 7
#define pitch_ki 0
#define pitch_kd 0

#define roll_kp 0
#define roll_ki 0
#define roll_kd 0

#define yaw_kp 0
#define yaw_ki 0
#define yaw_kd 0

void Control_System::rollPID(float current_roll, float *pid_roll, float *previous_integral_error_roll, float *previousError_roll,float sampleTime, int8_t desired_roll)
{
	//Instanciate object
	communication_setup Communicate;

	float previous_integral_error_print=0;
	float previousError_roll_print=0;

	previous_integral_error_print=*previous_integral_error_roll;
	previousError_roll_print=*previousError_roll;

	float	P_term,I_term=0, D_term;
	float	error_roll = desired_roll - current_roll;
	int16_t pid_roll_print=0;

	if(sampleTime>0.2)
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = roll_kp*error_roll; //proportional
	I_term = previous_integral_error_print+roll_ki*(error_roll*sampleTime);
	D_term = roll_kd * ((error_roll-previousError_roll_print)/sampleTime);

	*previousError_roll = error_roll;
	*previous_integral_error_roll=I_term;

	if (I_term < iMin)
	{
		I_term=iMin;
		//*previous_integral_error_roll=0;
	}
	if (I_term > iMax)
	{
		I_term=iMin;
		//*previous_integral_error_roll=0;
	}
	if(D_term > 120)
	{
		D_term=120;
	}
	if(D_term < -120)
	{
		D_term=-120;
	}

	*pid_roll = P_term+I_term+D_term;
	pid_roll_print=*pid_roll;

	Communicate.print(pid_roll_print);//roll
	Communicate.printString("\t");
	//printString("\r\n");
	//change rotor1&3
}
void Control_System::pitchPID(float current_pitch, float *pid_pitch, float *previous_integral_error_pitch,float *previousError_pitch,float sampleTime, int8_t desired_pitch)
{
	//Instanciate object
	communication_setup Communicate;

	float previous_integral_error_print=0;
	float previousError_pitch_print=0;

	previous_integral_error_print=*previous_integral_error_pitch;
	previousError_pitch_print=*previousError_pitch;

	float	 P_term=0, I_term=0, D_term=0;
	float	 error_pitch = desired_pitch - current_pitch;
	int16_t pid_pitch_print=0;

	if(sampleTime>0.2)
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = pitch_kp*error_pitch; //proportional
	I_term = previous_integral_error_print+pitch_ki*(error_pitch*sampleTime);
	D_term = pitch_kd * ((error_pitch-previousError_pitch_print )/sampleTime);

	*previousError_pitch = error_pitch;
	*previous_integral_error_pitch=I_term;

	if (I_term> iMax)
	{
		I_term= iMax;
		//*previous_integral_error_pitch=0;
	}
	if (I_term < iMin)
	{
		I_term = iMin;
		//*previous_integral_error_pitch=0;
	}
	if(D_term > 250)
	{
		D_term=250;
	}
	if(D_term < -250)
	{
		D_term=-250;
	}

	*pid_pitch = P_term+I_term+D_term;
	pid_pitch_print=*pid_pitch;

	Communicate.print(pid_pitch_print);//pitch
	Communicate.printString("\t");
	///print(D_term);
	//printString("\r\n");
	//change rotor1&2
}
void Control_System::yawPID(float current_yaw, float *pid_yaw, float *previous_integral_error_yaw,float *previousError_yaw,float sampleTime, int8_t desired_yaw)
{
	//Instanciate object
	communication_setup Communicate;

	float previous_integral_error_print=0;
	float previousError_yaw_print=0;

	previous_integral_error_print=*previous_integral_error_yaw;
	previousError_yaw_print=*previousError_yaw;

	float	P_term=0, I_term=0, D_term=0;
	float	error_yaw = desired_yaw - current_yaw;
	int16_t pid_yaw_print=0;

	if(sampleTime > 0.2 )
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = yaw_kp*error_yaw; //proportional
	I_term = previous_integral_error_print+yaw_ki*(error_yaw*sampleTime);
	D_term = yaw_kd*((error_yaw-previousError_yaw_print )/sampleTime);

	*previousError_yaw = error_yaw;
	*previous_integral_error_yaw=I_term;

	//Check for Integral Windup and Filter Derivative term
	if (I_term> iMax)
	{
		I_term= iMax;
	}
	if (I_term < iMin)
	{
		I_term = iMin;
	}
	if (D_term > 150)
	{
		D_term = D_term/2;
	}
	if (D_term < -150)
	{
		D_term = D_term/2;
	}
	if(D_term > 250)
	{
		D_term=250;
	}
	if(D_term < -250)
	{
		D_term=-250;
	}

	*pid_yaw = P_term+I_term+D_term;
	pid_yaw_print=*pid_yaw;

	Communicate.print(pid_yaw_print);//yaw
	Communicate.printString("\t");
	//print(D_term);
}
