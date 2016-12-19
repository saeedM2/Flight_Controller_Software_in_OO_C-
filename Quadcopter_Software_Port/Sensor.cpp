#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
//#include <map>
#include "Sensor.h"

int Sensor::readData(uint8_t Register_Address){
    //select the register to read from
	I2C0_MSA_R=(0b11010000);// select write mode
	I2C0_MDR_R=Register_Address;//write the register to read from
	I2C0_MCS_R=0x00000003;////run start
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent
	I2C0_MSA_R=(0b11010001);// select read mode
	I2C0_MCS_R=0x00000007;//run  start stop
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent
	return (I2C0_MDR_R);
}

void Sensor::write(uint8_t Register_Address, uint16_t data){
	I2C0_MSA_R=(0b11010000);// select write mode
	I2C0_MDR_R= Register_Address;//select the register
	I2C0_MCS_R=0x00000003;//run start
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent
	I2C0_MDR_R=data;///Write your bit field setting (data) to the corresponding registe address
	I2C0_MCS_R= 0x00000005;//run  stop
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent
}

int Magne::readData(uint8_t Register_Address){
	//select the register to read from
	I2C3_MSA_R=(0x3C);// select write_I2C mode
	I2C3_MDR_R=Register_Address;//write_I2C the register to read from
	I2C3_MCS_R=0x00000003;////run start
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	I2C3_MSA_R=(0x3D);// select read mode
	I2C3_MCS_R=0x00000007;//run  start stop
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	return (I2C3_MDR_R);
}

void Magne::write(uint8_t Register_Address, uint16_t data){
	I2C3_MSA_R=(0x3C);// select write_I2C mode
	I2C3_MDR_R= Register_Address;//select the register
	I2C3_MCS_R=0x00000003;//run start
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	I2C3_MDR_R=data;///write_I2C your bit field setting (data) to the corresponding registe address
	I2C3_MCS_R= 0x00000005;//run  stop
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent
}

void Config::MPU6050(void){
	//********Gyroscope and Accelerometer***********
	//intanciate object
	Accel configure_MPU6050;
	//Reset registers
	configure_MPU6050.write(0x6B, 0b10000000);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x19, 0x00);
	configure_MPU6050.write(0x1A, 0x00);
	configure_MPU6050.write(0x1C, 0x00);
	configure_MPU6050.write(0x1B, 0x00);
	configure_MPU6050.write(0x6B, 0x00);
	//Register 107 – Power Management 1//reset device
	configure_MPU6050.write(0x6B, 0b10000000);
	//(1) Register 25 – Sample Rate Divider// Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	configure_MPU6050.write(0x19, 0x00);
	//(2) Register 25 – Sample Rate Divider// Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	configure_MPU6050.write(0x19, 0x03);
	//(3) Register 25 – Sample Rate Divider// Set Gyro Full Scale Range
	configure_MPU6050.write(0x19, 2<<3);
	//(4) Register 25 – Sample Rate Divider// Set Accelerometer Full Scale
	configure_MPU6050.write(0x19, 3<<3);
	//(2) Register 25 – Sample Rate Divider// 41 Hz Acc filtering
	configure_MPU6050.write(0x19, 0x03);
	//(5) Register 25 – Sample Rate Divider// SMPLRT_DIV = 0
	configure_MPU6050.write(0x19, 0x00);
	//Register 26 – Configuration// disable input pin for data sync
	configure_MPU6050.write(0x1A, 0x06);
	//Register 28 –  Configure Accelerometer (Full Scale = +/-2G)
	configure_MPU6050.write(0x1C, 0x00<< 4);
	//Register 27 – Gyroscope Configuration ( FS_SEL: 0; Full Scale Range: ± 250 °/s; LSB Sensitivity: 131 LSB/°/s)
	configure_MPU6050.write(0x1B, 0x00<< 4);
	//Register 107 – Power Management 1// turn sleep mode off
	configure_MPU6050.write(0x6B, 0x00);
}

void Config::HMC5983(void){
	Magne configure_HMC5983;
	//Configuration A Register
	configure_HMC5983.write(0x00, 0b11011000);
	//Configutation B Register
	configure_HMC5983.write(0x01, 0b00000000);
	//Mode Register
	configure_HMC5983.write(0x02, 0b00000000);
}

void Config::Caliberate_Gyro_sensor(void){

	int16_t GYRO_XOUT_OFFSET=0;
	int16_t GYRO_YOUT_OFFSET=0;
	int16_t GYRO_ZOUT_OFFSET=0;

	int16_t Gyroscopex=0;
	int16_t Gyroscopey=0;
	int16_t Gyroscopez=0;

	int16_t GYRO_XOUT_OFFSET_1000SUM=0;
	int16_t GYRO_YOUT_OFFSET_1000SUM=0;
	int16_t GYRO_ZOUT_OFFSET_1000SUM=0;

	//intanciate object
	Gyro Gyroscope;

	for(int x = 0; x<1000; x++){
		Gyroscopex=Gyroscope.dir('x');
		Gyroscopey=Gyroscope.dir('y');
		Gyroscopez=Gyroscope.dir('z');

		GYRO_XOUT_OFFSET_1000SUM=GYRO_XOUT_OFFSET_1000SUM+Gyroscopex;
		GYRO_YOUT_OFFSET_1000SUM=GYRO_YOUT_OFFSET_1000SUM+Gyroscopey;
		GYRO_ZOUT_OFFSET_1000SUM=GYRO_ZOUT_OFFSET_1000SUM+Gyroscopez;
	}
	GYRO_XOUT_OFFSET=GYRO_XOUT_OFFSET_1000SUM/1000;
	GYRO_YOUT_OFFSET=GYRO_YOUT_OFFSET_1000SUM/1000;
	GYRO_ZOUT_OFFSET=GYRO_ZOUT_OFFSET_1000SUM/1000;
	Gyroscope.set_offset(GYRO_XOUT_OFFSET, GYRO_YOUT_OFFSET, GYRO_ZOUT_OFFSET);
}

int16_t Accel::dir(char dir){
	uint8_t Register_Address_H;
	uint8_t Register_Address_L;
	switch(dir){
		case 'x':
		case 'X':
			Register_Address_H=0x3B;
			Register_Address_L=0x3C;
			break;
		case 'y':
		case 'Y':
			Register_Address_H=0x3D;
			Register_Address_L=0x3E;
			break;
		case 'z':
		case 'Z':
			Register_Address_H=0x3F;
			Register_Address_L=0x40;
			break;
	}
	uint8_t accelerometer_low=0;
	uint8_t accelerometer_High=0;
	int16_t accelerometer=0;
	accelerometer_High=this->readData(Register_Address_H);
	accelerometer_low=this->readData(Register_Address_L);
	accelerometer=accelerometer_High<<8 | accelerometer_low;
	return accelerometer;
}

int16_t Gyro::dir(char dir){
	uint8_t Register_Address_H;
	uint8_t Register_Address_L;
	int16_t GYRO_OUT_OFFSET;
	switch(dir){
		case 'x':
		case 'X':
			Register_Address_H=0x43;
			Register_Address_L=0x44;
			GYRO_OUT_OFFSET=this->GYRO_XOUT_OFFSET;
			break;
		case 'y':
		case 'Y':
			Register_Address_H=0x45;
			Register_Address_L=0x46;
			GYRO_OUT_OFFSET=this->GYRO_XOUT_OFFSET;
			break;
		case 'z':
		case 'Z':
			Register_Address_H=0x47;
			Register_Address_L=0x48;
			GYRO_OUT_OFFSET=this->GYRO_XOUT_OFFSET;
			break;
	}
	uint8_t Gyroscope_low=0;
	uint8_t Gyroscope_High=0;
	int16_t Gyroscope=0;
	Gyroscope_High=this->readData(Register_Address_H);
	Gyroscope_low=this->readData(Register_Address_L);
	Gyroscope=(Gyroscope_High<<8 | Gyroscope_low) - GYRO_OUT_OFFSET;
	return Gyroscope;
}

const int16_t& Gyro::get_offset(char dir)const{
	int16_t result;
	switch(dir){
			case 'x':
			case 'X':
				result = this->GYRO_XOUT_OFFSET;
			case 'y':
			case 'Y':
				result = this->GYRO_YOUT_OFFSET;
			case 'z':
			case 'Z':
				result = this->GYRO_ZOUT_OFFSET;
		}
	return result;
}

void Gyro::set_offset(const int16_t &x,const int16_t &y, const int16_t &z ){
	this->GYRO_XOUT_OFFSET=x;
	this->GYRO_YOUT_OFFSET=y;
	this->GYRO_ZOUT_OFFSET=z;
}

Magne::Magne():Mx_offset(143), My_offset(-482.50), Mz_offset(-482.50),
		       Scalex(0.9652), Scaley(1.0184), Scalez(0.9040){};

int16_t Magne::dir(char dir){
	uint8_t Register_Address_H;
	uint8_t Register_Address_L;
	float Scale;
	float M_offset;

	switch(dir){
		case 'x':
		case 'X':
			Register_Address_H=0x03;
			Register_Address_L=0x04;
			Scale=this->Scalex;
			M_offset=this->Mx_offset;
			break;
		case 'y':
		case 'Y':
			Register_Address_H=0x07;
			Register_Address_L=0x08;
			Scale=this->Scaley;
			M_offset=this->My_offset;
			break;
		case 'z':
		case 'Z':
			Register_Address_H=0x05;
			Register_Address_L=0x06;
			Scale=this->Scalez;
			M_offset=this->Mz_offset;
			break;
	}
	uint8_t Magnetometer_low=0;
	uint8_t Magnetometer_High=0;
	int16_t Magnetometer=0;
	Magnetometer_High=this->readData(Register_Address_H);
	Magnetometer_low=this->readData(Register_Address_L);
	Magnetometer=(Magnetometer_High<<8 | Magnetometer_low);

	//Caliberate (Soft and Hard Distortion)
	Magnetometer=Scale*(Magnetometer-M_offset);
	return Magnetometer;
}
