#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

class communication_setup{

public:
	void i2c_Recover_from_bus_hang(void);
	void MPU6050_i2c(void);
	void HmC5983_i2c(void);
	void UART(void);
	void Transmit(uint16_t raw_data);//Transmit to Serial Monitor
	void print(int16_t data);
	void printString(char * string);
	void Recieve(uint16_t *raw_data);
};

#endif /* COMMUNICATION_H_ */
