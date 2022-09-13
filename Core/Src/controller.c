/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "pid.h"
#include "irs.h"
#include "encoders.h"
#include "utility.h"
#include "solver.h"
#include <math.h>

extern int16_t goal_forward_left;
extern int16_t goal_forward_right;

const int move_counts = 612; // 612
const int turn_counts = 456; // 456
const int init_counts = 300;

int distance = 0;

void move(int8_t n) {	// Move n cells forward (with acceleration)

	setState(MOVING);

	setPIDGoalA(0);
	setPIDGoalD(move_counts*n);

	while(!PIDdone())
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
	}

	resetPID();

}

void turn(int8_t n) {	// Make n 90 degree turns (no acceleration)

	setState(TURNING);

	setPIDGoalD(0);
	setPIDGoalA(turn_counts*n);

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
	setPIDGoalD(10000);

	resetEncoders(); // TODO: IS THIS NECESSARY??

	int16_t explore_done = 0;

	while(!explore_done)
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));

		if (fabs(((getLeftEncoderCounts() + getRightEncoderCounts())/2) - move_counts) < 20)
		{
			Action nextMove = solver(DEAD);
			switch(nextMove)
			{
				case FORWARD:
					resetEncoders();
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
