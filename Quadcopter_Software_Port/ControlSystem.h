#ifndef CONTROLSYSTEM_H_
#define CONTROLSYSTEM_H_

class Control_System {
public:
	void yawPID(float current_yaw,
			float *const pid_yaw,
			float *const previous_integral_error_yaw,
			float *const previousError_yaw,
			float sampleTime,
			int8_t desired_yaw);
	void pitchPID(float current_pitch,
			float *const pid_pitch,
			float *const previous_integral_error_pitch,
			float *const previousError_pitch,
			float sampleTime,
			int8_t desired_pitch);
	void rollPID(float current_roll,
			float *const pid_roll,
			float *const previous_integral_error_roll,
			float *const previousError_roll,
			float sampleTime,
int8_t desired_roll);
};

#endif /* CONTROLSYSTEM_H_ */
