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
	EXPLORING = 3
}STATE;

void setIRAngleOffset(int16_t angleOffset, int16_t leftGoal, int16_t rightGoal);
void setIRAngle(float left, float right);
void setState(STATE curr_state);

void resetPID(void);
void updatePID(void);
void setPIDGoalD(int16_t distance);
void setPIDGoalA(int16_t angle);
int8_t PIDdone(); // There is no bool type in C. True/False values are represented as 1 or 0.


#endif /* INC_PID_H_ */
