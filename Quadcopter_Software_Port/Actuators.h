#ifndef ACTUATORS_H_
#define ACTUATORS_H_

class Actuators {
public:
	void PWM_initilization(void);
	void PWM_adjustment(float pid_pitch, float pid_roll, float pid_yaw,uint16_t baseSpeed);
};

#endif /* ACTUATORS_H_ */
