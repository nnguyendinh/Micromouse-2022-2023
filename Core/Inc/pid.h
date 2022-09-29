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
	ACCELERATING = 4,
	CURVING = 5,
}STATE;

// Constants
#define kPw 0.001	// 0.01
#define kDw 0.000	// 0.0002
#define kPx 0.001	// 0.001
#define kDx 0.0000		//0.0

#define FRONT_kPx 0.6
#define FRONT_kPw 0.35

#define kPir 0.0203
#define kPir2 0.0103

#define ACC_CONSTANT 0.0015 // 0.002

#define PWM_MAX_X 0.7 // 0.65
#define PWM_MAX_W 0.4
#define PWM_MIN_X 0.35
#define PWM_MIN_W 0.35
#define PWM_MIN 0.3		// TODO: ADD TO MOTORS.C

#define EXPLORE_SPEED 0.7	// in mm/s

#define OUTER_SPEED 1.095833
#define INNER_SPEED 0.404167

// Functions
void setPIDGoalD(int16_t distance);
void setPIDGoalA(int16_t angle);
void setState(STATE curr_state);
void setIRGoals(int16_t frontLeftGoal, int16_t frontRightGoal, int16_t leftGoal, int16_t rightGoal);
void setIRDistance(int16_t curr_forward_left, int16_t curr_forward_right);
void setIRAngle(float left, float right);

void updatePID(void);
int8_t PIDdone();
void resetPID(void);


#endif /* INC_PID_H_ */
