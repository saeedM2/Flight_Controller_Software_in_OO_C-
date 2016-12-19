#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "Sensor.h"
#include "Communication.h"
#include "Filter.h"
#include "Timer.h"
#include "Actuators.h"
#include "ControlSystem.h"

//Flag for UART1 interrupt
#define FLAG_NONE 0x00000000
#define FLAG_Data_Recieved_UART1 0x00000001
#define FLAG_ENTER_PRESSED '\r'
volatile uint8_t flag=FLAG_NONE;

//extern "C" prevents name mangling between C and C++
extern "C" void UART1_Handler(void){
	int8_t read=0;
	while ((UART1_FR_R & (1<<4)) !=0);
	read=UART1_DR_R;

		if(UART1_DR_R =='\r')
		{
			flag=FLAG_ENTER_PRESSED;
			//clears the interrupt flag for UART for aknowledgement
			UART1_ICR_R |=(1<<4);//Receive Interrupt Clear
		}
		else if(UART1_DR_R !='\r')
		{
			flag=FLAG_Data_Recieved_UART1;
			//clears the interrupt flag for UART for aknowledgement
			UART1_ICR_R |=(1<<4);//Receive Interrupt Clear
		}
}

int main(void) {

	//SYSCTL_RCC_R=SYSCTL_DC1_MINSYSDIV_66 | 0x0 | SYSCTL_RCC_XTAL_20MHZ |SYSCTL_RCC_OSCSRC_MAIN;
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|0x0|SYSCTL_RCC_XTAL_20MHZ|SYSCTL_RCC_OSCSRC_MAIN);

	//initialize variables
	int16_t accelerometerx=0;
	int16_t accelerometery=0;
	int16_t accelerometerz=0;
	int16_t Gyroscopex=0;
	int16_t Gyroscopey=0;
	int16_t Gyroscopez=0;
	int16_t Magnetometerx=0;
	int16_t Magnetometery=0;
	int16_t Magnetometerz=0;
	int16_t GYRO_XRATE_degrees_per_second=0;
	int16_t GYRO_YRATE_degrees_per_second=0;
	int16_t GYRO_ZRATE_degrees_per_second=0;
	int16_t ACCEL_XANGLE=0;
	int16_t ACCEL_YANGLE=0;

	uint16_t U=0;//used to intilize an important flag
	float pitch=0;
	float roll=0;
	float yaw=0;
	float Yaw_filtered=0;
	float RawMagx=0;
	float RawMagy=0;
	float RawMagz=0;
	float initialYaw=0;

	int16_t pitch_conversion=0;
	int16_t roll_conversion=0;
	int16_t yaw_conversion=0;

	float pid_pitch=0; //pitch pid controller
	float pid_roll=0;
	float pid_yaw=0;
	float previousError_pitch=0;
	float previousError_roll=0;
	float previousError_yaw=0;
	float previous_integral_error_pitch=0;
	float previous_integral_error_roll=0;
	float previous_integral_error_yaw=0;

	float sampleTime=0;
	float SampleTime_START=0;
	float SampleTime_END=0;
	float SampleTime_END_converted=0;
	uint16_t k= 0; //Must be 16 bits signed
	int16_t raw_data=0;

	int8_t array[4]={0};
	uint8_t i=0;
	uint8_t n=0;

	//Instanciate Objects:
	communication_setup Communicate;
	Config Configure;
	Accel Accelerometer;
	Gyro Gyroscope;
	Magne Magnetometer;
	Filter Filter_Data;
	Timer Stop_watch;
	Control_System Control_system_Q;
	Actuators motor;

	//Ensure The I2C recovers incase it hangs due to slave pulling data line down
	Communicate.i2c_Recover_from_bus_hang();
	//initlizate the i2c0 and i2c3 peripheral for IMU
	Communicate.MPU6050_i2c();
	//initlizate the i2c0 and i2c3 peripheral for magnetometer
	Communicate.HmC5983_i2c();
	//initilize UART0 and UART1 for serial communication
	Communicate.UART();
	//Configure MPU6050 IMU sensor (settings chosen within the functions)
	Configure.MPU6050();
	//Configure HMC5983 sensor
	Configure.HMC5983();
	//retrieve the caliberation Gyroscope offset values for next set of calculations (used for calculating Euler angles)
	Configure.Caliberate_Gyro_sensor();
	//PWM initilizations
	motor.PWM_initilization();
	//Re-initilize timer
	Stop_watch.initilize_Timer();

	while(1){
		//Start Timer
		SampleTime_START=TIMER0_TAR_R;//time given in seconds

		accelerometerx = Accelerometer.dir('x');
		accelerometery = Accelerometer.dir('y');
		accelerometerz = Accelerometer.dir('z');
		Gyroscopex = Gyroscope.dir('x');
		Gyroscopey = Gyroscope.dir('y');
		Gyroscopez = Gyroscope.dir('z');
		Magnetometerx=Magnetometer.dir('x');
		Magnetometery=Magnetometer.dir('y');
		Magnetometerz=Magnetometer.dir('z');

		//Temporary Auxillary Variable
		RawMagx=(float)Magnetometerx;
		RawMagy=(float)Magnetometery;
		RawMagz=(float)Magnetometerz;

		//calculate the roll and pitch using accelerometer!
		Filter_Data.Euler_acceleromter_Angle(accelerometerx,
											accelerometery,
											accelerometerz,
											&ACCEL_XANGLE,
											&ACCEL_YANGLE);

		//calculate the Final gyrscope angle rate by dividing by the sensitivity listed in the data sheet
		Filter_Data.Convert_Gyro_Rates_to_degrees_per_second(Gyroscopex,
															Gyroscopey,
															Gyroscopez,
															&GYRO_XRATE_degrees_per_second,
															&GYRO_YRATE_degrees_per_second,
															&GYRO_ZRATE_degrees_per_second);

		//calculate the roll and pitch using sensor fusion (compplementary filter)
		Filter_Data.ComplementaryFilter(GYRO_XRATE_degrees_per_second,
										GYRO_YRATE_degrees_per_second,
										GYRO_ZRATE_degrees_per_second,
										&pitch,
										&roll,
										ACCEL_XANGLE,
										ACCEL_YANGLE,
										SampleTime_END_converted);

		//convert data (for Printing on Terminal)
		pitch_conversion=(int16_t)pitch;
		roll_conversion=(int16_t)roll;

		//Calculate Yaw and Perfrom Tilt compnesation
		Filter_Data.Tilt_compensation(roll,
									pitch,
									RawMagx,
									RawMagy,
									RawMagz,
									&yaw);

		//wait to stabilize magnet data
		if(U==150)
		{
			initialYaw=yaw;
		}
		yaw=yaw-initialYaw;

		//Fuse Magnetometer data and gyroscope data to estimate Yaw with minimum drift//The output for this function is "Yaw_filtered"
		Filter_Data.Calculate_Yaw_Filtered(yaw,
											GYRO_ZRATE_degrees_per_second,
											&Yaw_filtered);
		//cast data
		yaw_conversion=(int16_t)Yaw_filtered;

		if(U==0){
		Communicate.printString("Roll");
		}
		Communicate.printString("\t");
		if(U==0){
		Communicate.printString("Pitch");
		}
		Communicate.printString("\t");
		if(U==0){
		Communicate.printString("Yaw");
		}
		Communicate.printString("\r\n");
		Communicate.print(roll_conversion);
		Communicate.printString("\t");
		Communicate.print(pitch_conversion);
		Communicate.printString("\t");
		Communicate.print(yaw_conversion);
		Communicate.printString("\t");

		if( U < 170){
			U++;//once set we never print "Roll" and "Pitch" and "Yaw"
		}

		// read the duty cyle for motor
		if( (flag == FLAG_Data_Recieved_UART1)){
		  raw_data=UART1_DR_R;
		  array[n]=raw_data-48;//convert from ASCII to raw numbers
		  n++;
		  flag=FLAG_NONE;
		}
		if(flag == FLAG_ENTER_PRESSED){
		   k= 0;
		   while(i<n) {
			   k = 10 * k + array[i];
			   i++;
		   }
		   raw_data=0;
		   array[0]=0;
		   array[1]=0;
		   array[2]=0;
		   array[3]=0;
		   i=0;
		   n=0;
		   flag=FLAG_NONE;
		 }

		//motor shut off
		PWM0_2_CMPA_R= k-1;//(RED)//motor1
		PWM0_0_CMPA_R= k-1;//(Yellow)//motor2
		PWM0_1_CMPA_R= k-1;//(ORANGE)//motor3
		PWM1_1_CMPA_R =k-1;//(green)//motor4

		//End counting time and now subtract the difference to find the elapsed time
		SampleTime_END=((TIMER0_TAR_R)-(SampleTime_START));
		SampleTime_END_converted=(SampleTime_END/(80000000));
		sampleTime=SampleTime_END_converted;

		//Take Corrective action ONLY is angles are reasonable
		 if(pitch_conversion >-90 && pitch_conversion <90 &&  roll_conversion >-90 && roll_conversion < 90){
			if(k>=38){
				int16_t desired_pitch =  2;
				int16_t desired_roll  = 0;
				int16_t desired_yaw=0;

				//Calculate PID gains
				if((-4 > pitch) || (pitch > 4)){
					//PID control functions
					Control_system_Q.pitchPID(pitch,&pid_pitch,&previous_integral_error_pitch,&previousError_pitch,sampleTime, desired_pitch);
				}
				if((-3 > roll)  || (roll > 3)){
					//PID control functions
					Control_system_Q.rollPID(roll, &pid_roll,&previous_integral_error_roll,&previousError_roll,sampleTime, desired_roll);
				}
				if((-4 > Yaw_filtered ) || (Yaw_filtered > 4)){
					//PID control functions
					Control_system_Q.yawPID(Yaw_filtered, &pid_yaw, &previous_integral_error_yaw,&previousError_yaw,sampleTime,desired_yaw);
				}

				//Adjust speed
				motor.PWM_adjustment(pid_pitch, pid_roll, pid_yaw,k);

				//Reset respective PID gains when conditions are met
				if((-4 < pitch) && (pitch < 4)){
					pid_pitch=0;//reset the variable
					previous_integral_error_pitch=0;
				}
				if((-4 < roll) && (roll < 4)){
					pid_roll=0;//reset the variable
					previous_integral_error_roll=0;
				}
				if((-4 < Yaw_filtered ) && (Yaw_filtered < 4)){
					pid_yaw=0;//reset the variable
					previous_integral_error_yaw=0;
				}
			}
		 }
	}
	return 0;
}
