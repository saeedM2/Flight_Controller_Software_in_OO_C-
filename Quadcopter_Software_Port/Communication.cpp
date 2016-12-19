#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"
#include "Communication.h"

void communication_setup::i2c_Recover_from_bus_hang(){
	//MPU6050 Recovery
	SYSCTL_RCGCGPIO_R = 0b00000010;//Clock to port B (pin 2 and 3 will have I2C0_SCL and I2C0_SDA respectively)
	GPIO_PORTB_DIR_R = 0b00000100;
	GPIO_PORTB_AFSEL_R &= 0b00000000;//Disable Alternative function
	GPIO_PORTB_DEN_R |= 0b00001100;//set pin 2 and 3 to be digital pins
	int8_t i=10;
	while(i>=0){
		GPIO_PORTB_DATA_R = 0b00000100;
		i--;
	}
	GPIO_PORTB_DATA_R &= 0b0000000;
	GPIO_PORTB_DATA_R |= 0b0001100;
	GPIO_PORTB_DATA_R &= 0b0000000;
	//HMC5983 Recovery
	SYSCTL_RCGCGPIO_R = (1<<3);//Clock to port B (pin 0 and 1 will have I2C0_SCL and I2C0_SDA respectively)
	GPIO_PORTD_DIR_R = 0b00000001;
	GPIO_PORTD_AFSEL_R &= 0b00000000;//Disable Alternative function
	GPIO_PORTD_DEN_R |= 0b00000011;//set pin 0 and 1 to be digital pins
	i=10;
	while(i>=0){
		GPIO_PORTD_DATA_R = 0b00000001;
		i--;
	}
	GPIO_PORTD_DATA_R &= 0b0000000;
	GPIO_PORTD_DATA_R |= 0b0000011;
	GPIO_PORTD_DATA_R &= 0b0000000;
}

void communication_setup::MPU6050_i2c() {
	//Reset the i2c bus
	int16_t i=10000;
	while(i>=0){
		SYSCTL_SRI2C_R=0b1111;
		i--;
	}
	i=10;
	while(i>=0){
		SYSCTL_SRI2C_R=0b0000;
		i--;
	}
	//initilize i2c0
	SYSCTL_RCGCI2C_R |= 0b00000001;//selected i2c module 0 (will connect to portB pin 2 and 3 with I2C0_SCL and I2C0_SDA respectively
	SYSCTL_RCGCGPIO_R |= 0b00000010;//Clock to port B (pin 2 and 3 will have I2C0_SCL and I2C0_SDA respectively)
	GPIO_PORTB_AFSEL_R |= 0b00001100;//pin 2 and 3 are assigned alternative function. Nanmely connecting to I2C0_SCL and I2C0_SDA respectively
	GPIO_PORTB_DEN_R |= 0b00001100;//set pin 2 and 3 to be digitcal pins
	GPIO_PORTB_ODR_R |= (1<<3);//set SDA pin to open drain (PB3M, PIN4 is set to high)
	GPIO_PORTB_PCTL_R |= 0x00003300;
	I2C0_MCR_R |= (1<<4);
	I2C0_MTPR_R |= 0x00000009;
}

void communication_setup::HmC5983_i2c() {
	//Initilize I2c3
	SYSCTL_RCGCI2C_R |=(1<<3);//selected i2c module 3 (D0 and D1)
	SYSCTL_RCGCGPIO_R |=(1<<3);//Clock to port D
	GPIO_PORTD_AFSEL_R |=0b00000011;//pin 0 and 1 are assigned alternative function.
	GPIO_PORTD_DEN_R |=0b00000011;//set pin 0 and 1 to be digitcal pins
	GPIO_PORTD_ODR_R |=(1<<1);//set SDA pin to open drain (bit 0 (pin 1) is set to high)
	GPIO_PORTD_PCTL_R |=0x00000033;
	I2C3_MCR_R |= (1<<4);
	I2C3_MTPR_R |=0x00000009;
}

void communication_setup::UART(){
	//UART0
	SYSCTL_RCGCUART_R |= (1<<0);
	SYSCTL_RCGCGPIO_R |= (1<<0);
	GPIO_PORTA_AFSEL_R|= (1<<1)|(1<<0);
	GPIO_PORTA_PCTL_R |= (1<<0)|(1<<4);
	GPIO_PORTA_DEN_R  |= (1<<0)|(1<<1);
	UART0_CTL_R &= ~(1<<0);
	UART0_IBRD_R = 7;//128000 bud rate
	UART0_FBRD_R = 52;//128000 bud rate
	UART0_LCRH_R = (0x3<<5);
	UART0_CC_R = 0x5;
	UART0_CTL_R = (1<<0)|(1<<8)|(1<<9);
	//UART1
	SYSCTL_RCGCUART_R |=0b00000010;//activated uart1 clock
	SYSCTL_RCGCGPIO_R |=0b00000010;
	GPIO_PORTB_AFSEL_R |=0b00000011; //set pin B0 and B1
	GPIO_PORTB_PCTL_R |=(1<<0)|(1<<4);//same as 0x00000011 as shown on Table 23.5
	GPIO_PORTB_DEN_R |=0b00000011;
	UART1_CTL_R &= ~(1<<0);
	UART1_IBRD_R |= 104;
	UART1_FBRD_R |=11;
	UART1_LCRH_R &=0b00000000;//no parity
	UART1_LCRH_R &=~(1<<3);//one stop bit selected
	UART1_LCRH_R |= (UART_LCRH_WLEN_8);
	UART1_CC_R=0X5;
	UART1_IM_R |= 0b00010000;//Recieve interrupt is enabled
	UART1_CTL_R |= 1<<0;//enable the uart
	UART1_CTL_R |=1<<8;//enable the transmit
	UART1_CTL_R |=1<<9;//enable the recieve
	NVIC_EN0_R |=0b01000000;//Set bit 6 for Directing interrupt from IRQ 6 to CPU
}

void communication_setup::Transmit(uint16_t raw_data){
	while(UART0_FR_R & (1<<5)){
		//This function transmitts to putty
		//keep looping until the transmission register is empty
	}
	UART0_DR_R =raw_data;
}

void communication_setup::Recieve(uint16_t *raw_data){
	while ((UART1_FR_R & (1<<4)) !=0);
	*raw_data=UART0_DR_R;
}

void communication_setup::printString(char * string){
	while(*string){
		this->Transmit(*(string)++);
	}
}

void communication_setup::print(int16_t data){
	int8_t array[5]={0};
	if(data < 0){
	data=data*(-1);
	printString("-");
	}
	int digit=0;
	int i=0;
	while(data > 0){
		 digit = (int16_t)data % 10;
		 array[i]=digit+48;//+48 because we need convert digits to a value that can be seen by putty
		 i++;
		 data /= 10;
	}
	if(data==0 & i==0){
		while((UART0_FR_R & (1<<5)) != 0);
		this->Transmit(0+48);
	}
	int n=i;
	while(n>-1){
		while((UART0_FR_R & (1<<5)) != 0);
		this->Transmit(array[n]);
		n--;
	}
}
