/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "motors.h"
#include "pid.h"
#include "irs.h"
#include "encoders.h"
#include "utility.h"
#include "solver.h"
#include <math.h>

extern int16_t goal_forward_left;
extern int16_t goal_forward_right;

void move(int8_t n) {	// Move n cells forward (with acceleration)

	setState(MOVING);

	setPIDGoalA(0);
	setPIDGoalD(MOVE_COUNTS*n);

	while(!PIDdone())
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
	}

	resetPID();

}

void turn(int8_t n) {	// Make n 90 degree turns (no acceleration)

	setState(TURNING);

	setPIDGoalD(0);
	setPIDGoalA(TURN_COUNTS*n);

	while(!PIDdone())
	{

	}

	resetPID();

}

void moveEncoderCount(int8_t n) {	// Move n encoder counts (with acceleration)

	setState(MOVING);

	setPIDGoalA(0);
	setPIDGoalD(n);

	while(!PIDdone())
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
	}

	resetPID();

}

void explore() {	// Move forward at a constant speed until a turn is needed

	setState(EXPLORING);

	setPIDGoalA(0);

	resetEncoders();	// TODO: IS THIS NECESSARY?

	int16_t explore_done = 0;

	while(!explore_done)
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));

		int16_t distance = (getLeftEncoderCounts() + getRightEncoderCounts())/2;

		if (distance % MOVE_COUNTS < 15 || distance % MOVE_COUNTS > MOVE_COUNTS - 15)
			// If distance is within 15 ticks of the cell distance
		{
			Action nextMove = solver(DEAD);
			switch(nextMove)
			{
				case FORWARD:
					break;
				case LEFT:
//					moveEncoderCount(move_counts/2);
					resetPID();
					turn(-1);
					explore_done = 1;
					break;
				case RIGHT:
//					moveEncoderCount(move_counts/2);
					resetPID();
					turn(1);
					explore_done = 1;
					break;
				case IDLE:
					explore_done = 1;
					break;
			}
		}
	}

	resetPID();

}

void frontCorrection() {

	setState(REST);

	int16_t forward_left = 0;
	int16_t forward_right = 0;

	while(!PIDdone())
	{
		forward_left = readIR(IR_FORWARD_LEFT);
		forward_right = readIR(IR_FORWARD_RIGHT);

		setIRDistance(forward_left, forward_right);

		if (fabs(forward_left - goal_forward_left) < 200 && fabs(forward_right - goal_forward_right) < 200)
		{
			break;
		}
	}

	resetPID();

}

void curve(int8_t n) {
//
//	setPIDGoalD(fabs(481 * n));	// 481
////	setPIDGoalA(TURN_COUNTS*n);
//	setPIDGoalA(0);
//
//	setLeftVelocity((n >= 0) ? outer_speed : inner_speed);
//	setRightVelocity((n >= 0) ? inner_speed : outer_speed);
//
//	setState(CURVING);
//
//	while(!PIDdone())
//	{
//
//	}
//
//	resetPID();

}
