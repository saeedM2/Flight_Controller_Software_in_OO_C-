#ifndef SENSOR_H_
#define SENSOR_H_

class Config {
	public:
		void MPU6050(void);
		void HMC5983(void);
		void Caliberate_Gyro_sensor(void);
};

class Sensor {
	public:
		virtual int16_t dir(char dir)=0;	//make direction purely virtual
		virtual void write(uint8_t Register_Address, uint16_t data);
		virtual int readData(uint8_t Register_Address);
};

class Accel: public Sensor {
	public:
		int16_t dir(char dir);
};

class Gyro: public Sensor {
	public:
		int16_t dir(char dir);

		void set_offset(const int16_t &x,const int16_t &y, const int16_t &z );
		const int16_t& get_offset(char dir)const;

	private:
		int16_t GYRO_XOUT_OFFSET;
		int16_t GYRO_YOUT_OFFSET;
		int16_t GYRO_ZOUT_OFFSET;
};

class Magne:public Sensor {
	public:
	int16_t dir(char dir);
	int readData(uint8_t Register_Address);
	void write(uint8_t Register_Address, uint16_t data);
	Magne();

	private:
		//Caliberation perameters (Calculated Using MATLAB):
		//Calibrate Against Hard-Iron Distortion
		uint16_t Mx_offset;
		float 	 My_offset;
		float	 Mz_offset;
		//Calibrate Against Soft-Iron Distortion
		float Scalex;
		float Scaley;
		float Scalez;
};
#endif /* SENSOR_H_ */
