/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "pid.h"
#include "irs.h"
#include "encoders.h"
#include "utility.h"

void move(int8_t n) {	// Move n cells forward (with acceleration)

	setPIDGoalA(0);
	setPIDGoalD(move_counts*n);

	setState(MOVING);

	while(!PIDdone())
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
	}

	resetPID();

}

void turn(int8_t n) {	// Make n 90 degree turns (no acceleration)

	setPIDGoalD(0);
	setPIDGoalA(turn_counts*n);

	setState(TURNING);

	while(!PIDdone())
	{

	}

	resetPID();

}

void moveEncoderCount(int8_t n) {	// Move n encoder counts (with acceleration)

	setPIDGoalA(0);
	setPIDGoalD(n);

	setState(MOVING);

	while(!PIDdone())
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
	}

	resetPID();

}

void explore() {	// Move forward at a constant speed until a turn is needed

	setPIDGoalA(0);

	setState(EXPLORING);

	resetEncoders();

	while()
	{
		setIRAngle(readIR(IR_LEFT), readIR(IR_RIGHT));
		/*
		 * if we are at a transition point:
		 * 		run floodfill/dead reckoning
		 * 		if we are turning left or right:
		 * 			run moveEncoderCount to get to center
		 * 			turn left or right
		 */

		if ((getLeftEncoderCounts() + getRightEncoderCounts())/2)
		{

		}
	}

	resetPID();

}
