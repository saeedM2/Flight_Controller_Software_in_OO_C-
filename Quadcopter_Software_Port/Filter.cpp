#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "Filter.h"


void  Filter::Euler_acceleromter_Angle(int16_t accelerometerx,
										int16_t accelerometery,
										int16_t accelerometerz,
										int16_t *const ACCEL_XANGLE,
										int16_t *const ACCEL_YANGLE ){

	*ACCEL_XANGLE = 57.295*atan((float)accelerometery/sqrt(pow((float)accelerometerz,2)+pow((float)accelerometerx,2)));
	*ACCEL_YANGLE = 57.295*atan((float)-accelerometerx/sqrt(pow((float)accelerometerz,2)+pow((float)accelerometery,2)));
}

void  Filter::Convert_Gyro_Rates_to_degrees_per_second(int16_t Gyroscopex,
														  int16_t Gyroscopey,
														  int16_t Gyroscopez,
														  int16_t *const GYRO_XRATE_degrees_per_second,
														  int16_t *const GYRO_YRATE_degrees_per_second,
														  int16_t *const GYRO_ZRATE_degrees_per_second){

	//131 is the sensitivity constant from Datasheet
	*GYRO_XRATE_degrees_per_second=(float)Gyroscopex*(0.007634);
	*GYRO_YRATE_degrees_per_second=(float)Gyroscopey*(0.007634);
	*GYRO_ZRATE_degrees_per_second=(float)Gyroscopez*(0.007634);

	//Note we did implement the sensitivity division for the accelerometer like we did above (for gyroscope)
	//because the atan2 function just relies on the ratio of the two for calulating the angles.
}

void Filter::Tilt_compensation(float roll,
								float pitch,
								float RawMagx,
								float RawMagy,
								float RawMagz,
								float *const yaw){

	//Speedup Calculations by Storing Repetitive Calculations:
	float cosRoll = cosf(roll*0.01745);
	float sinRoll = sinf(roll*0.01745);
	float cosPitch =cosf(pitch*0.01745);
	float sinPitch =sinf(pitch*0.01745);

	//Calculate tilt Compensated Yaw
	float Y_h = (RawMagx*sinRoll*sinPitch) + (RawMagy*cosRoll) - (RawMagz*sinRoll*cosPitch);
	float X_h = (RawMagx*cosPitch) + (RawMagz*sinPitch);

	*yaw=57.295*atan2(Y_h, X_h);
	//*yaw=57.295*atan(Y_h / X_h);
	/*	if (Heading < 0)
	Heading += 360;*/
}

void Filter::Calculate_Yaw_Filtered(float yaw,
									  int16_t GYRO_ZRATE_degrees_per_second,
									  float *const Yaw_filtered){

	*Yaw_filtered= *Yaw_filtered+GYRO_ZRATE_degrees_per_second*0.0030;
	*Yaw_filtered=  (*Yaw_filtered)*(0.98) + (yaw)*(0.02);
}

void Filter::ComplementaryFilter(int16_t GYRO_XRATE_degrees_per_second,
									int16_t GYRO_YRATE_degrees_per_second,
									int16_t  GYRO_ZRATE_degrees_per_second,
									float *const pitch,
									float *const roll,
									int16_t ACCEL_XANGLE,
									int16_t ACCEL_YANGLE,
									float dt ){

	//(1) calculate the roll and pitch without any compnesation from the accelerometer data
	*roll=*roll +(float)GYRO_XRATE_degrees_per_second*0.0030; // Angle around the X-axis
	*pitch=*pitch+(float)GYRO_YRATE_degrees_per_second*0.0030;// Angle around the Y-axis

	//(2) Next, calcualate the compensated (more accurate) data using the complementary filter (sensor fusion)
	*roll=*roll*0.98+ACCEL_XANGLE*0.02;
	*pitch=*pitch*0.98+ACCEL_YANGLE*0.02;
}
