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
	CURVING = 5
}STATE;

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
