#ifndef FILTER_H_
#define FILTER_H_

class Filter {
public:
	void Euler_acceleromter_Angle(int16_t accelerometerx,
								int16_t accelerometery,
								int16_t accelerometerz,
								int16_t *const ACCEL_XANGLE,
								int16_t *const ACCEL_YANGLE );

	void Convert_Gyro_Rates_to_degrees_per_second(int16_t Gyroscopex,
												  int16_t Gyroscopey,
												  int16_t Gyroscopez,
												  int16_t *const GYRO_XRATE_degrees_per_second,
												  int16_t *const GYRO_YRATE_degrees_per_second,
												  int16_t *const GYRO_ZRATE_degrees_per_second);

	void Tilt_compensation(float roll,
						float pitch,
						float RawMagx,
						float RawMagy,
						float RawMagz,
						float *const yaw);

	void Calculate_Yaw_Filtered(float yaw,
								int16_t GYRO_ZRATE_degrees_per_second,
								float *const Yaw_filtered);

	void ComplementaryFilter(int16_t GYRO_XRATE_degrees_per_second,
							int16_t GYRO_YRATE_degrees_per_second,
							int16_t  GYRO_ZRATE_degrees_per_second,
							float *const pitch,
							float *const roll,
							int16_t ACCEL_XANGLE,
							int16_t ACCEL_YANGLE,
							float dt );
};

#endif /* FILTER_H_ */
