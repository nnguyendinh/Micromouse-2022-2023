/*
 * motors.h
 */

#include "main.h"

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#define PWM_MAX 0.8 // You probably shouldn't mess with this, but if you do, do not exceed 1
#define MAX_TIMER_COUNTS 3199 // Don't change this either

float limitPWM(float pwm);
void setMotorRPWM(float pwm);
void setMotorLPWM(float pwm);
void resetMotors();

#endif /* INC_MOTORS_H_ */
