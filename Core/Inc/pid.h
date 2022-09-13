/*
 * pid.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef enum
{
	REST = 0,
	MOVING = 1,
	TURNING = 2,
	EXPLORING = 3,
	ACCELERATING = 4
}STATE;

// Constants
#define kPw 0.001	// 0.01
#define kDw 0.000	// 0.0002
#define kPx 0.001	// 0.001
#define kDx 0.0000		//0.0

#define front_kPx 0.6
#define front_kPw 0.35

#define kPir 0.0203
#define kPir2 0.0103

#define xacceleration 0.005 // 0.002
#define xaccelerationTEST 0.005

#define PWM_max_x 0.7 // 0.65
#define PWM_max_w 0.4
#define PWM_min_x 0.35
#define PWM_min_w 0.35
#define PWM_min 0.3		// TODO: ADD TO MOTORS.C

#define explore_speed 0.3

// Functions
void setIRGoals(int16_t frontLeftGoal, int16_t frontRightGoal, int16_t leftGoal, int16_t rightGoal);
void setIRDistance(int16_t curr_forward_left, int16_t curr_forward_right);
void setIRAngle(float left, float right);
void setState(STATE curr_state);

void resetPID(void);
void updatePID(void);
void setPIDGoalD(int16_t distance);
void setPIDGoalA(int16_t angle);
int8_t PIDdone();


#endif /* INC_PID_H_ */
