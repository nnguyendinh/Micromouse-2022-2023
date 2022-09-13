/*
 * motors.c
 */

#include "motors.h"
#include "pid.h"

extern float velocity_left;
extern float velocity_right;

float limitPWM(float pwm) {
	if (pwm > PWM_MAX)
		return PWM_MAX;
	else if (pwm < -PWM_MAX)
		return -PWM_MAX;
	else
		return pwm;
}

void setMotorLPWM(float pwm) {
	if (pwm >= 0)
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = (uint32_t) - (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}

}

void setMotorRPWM(float pwm) {
	if (pwm >= 0)
	{
		TIM1->CCR4 = 0;
		TIM1->CCR3 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else
	{
		TIM1->CCR3 = 0;
		TIM1->CCR4 = (uint32_t) - (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void resetMotors() {
	setMotorLPWM(0);
	setMotorRPWM(0);
}

void setLeftVelocity(float v) {

	velocity_left = v;
	setState(ACCELERATING);
}
void setRightVelocity(float v) {

	velocity_right = v;
	setState(ACCELERATING);

}


