/*
 * pid.c
 */

#include <math.h>
#include "pid.h"
#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "irs.h"
#include "utility.h"

// Goal Parameters
float goal_distance = 0;
float goal_angle = 0;

extern int16_t goal_forward_left;
extern int16_t goal_forward_right;
extern int16_t goal_left;
extern int16_t goal_right;

extern float velocity_left;
extern float velocity_right;

int16_t IRAngleOffset = 0;

// Error Parameters
float angleError = 0;
float oldAngleError = 0;
float oldAngleErrors[10] = {0};
float angleCorrection = 0;
float derivativeAngleCorrection = 0;
float proportionalAngleCorrection = 0;

float distanceError = 0;
float oldDistanceError = 0;
float oldDistanceErrors[10] = {0};
float distanceCorrection = 0;
float oldDistanceCorrection = 0;

float left_error = 0;
float old_left_error = 0;
float old_left_errors[10] = {0};

float right_error = 0;
float old_right_error = 0;
float old_right_errors[10] = {0};

float IRadjustment = 0;

float test1 = 0;
float test2 = 0;

// Miscellaneous
STATE state = REST;
float left_PWM_value = 0;
float right_PWM_value = 0;
int goal_reached_timer = 0;

void setPIDGoalD(int16_t distance) { goal_distance = distance; }
void setPIDGoalA(int16_t angle) { goal_angle = angle; }
void setState(STATE curr_state) { state = curr_state; }

void setIRGoals(int16_t frontLeftGoal, int16_t frontRightGoal, int16_t leftGoal, int16_t rightGoal) {

	IRAngleOffset = leftGoal - rightGoal;
	goal_forward_left = frontLeftGoal;
	goal_forward_right = frontRightGoal;
	goal_left = leftGoal;
	goal_right = rightGoal;

}

void setIRDistance(int16_t curr_forward_left, int16_t curr_forward_right) {

	setPIDGoalA(front_kPw * ((goal_forward_left - curr_forward_left) - (goal_forward_right - curr_forward_right)));
	setPIDGoalD(front_kPx * ((goal_forward_left - curr_forward_left + goal_forward_right - curr_forward_left)/2));

}

// TODO: CHANGE TO USE WALL CHECK FUNCTIONS
void setIRAngle(float left, float right){

	if (left > 600 && right > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir * ((left - right) - IRAngleOffset));
	}
	else if (left > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir2 * (left - goal_left));
	}
	else if (right > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir2 * (goal_right - right));
	}
	else
		IRadjustment = 0;
}

void resetPID() {

//////////////	RESET ALL ANGLE AND DISTANCE ERRORS TO 0	////////////////
	angleError = 0;
	oldAngleError = 0;
	angleCorrection = 0;

	for (int i = 0; i < 10; i++)
		oldAngleErrors[i] = 0;

	distanceError = 0;
	oldDistanceError = 0;
	distanceCorrection = 0;

	for (int i = 0; i < 10; i++)
		oldDistanceErrors[i] = 0;

	left_error = 0;
	old_left_error = 0;
	for (int i = 0; i < 10; i++)
		old_left_errors[i] = 0;

	right_error = 0;
	old_right_error = 0;
	for (int i = 0; i < 10; i++)
		old_right_errors[i] = 0;

	IRadjustment = 0;

////////////// 	RESET ALL GOALS AND ENCODER COUNTS TO 0 	///////////////////
	goal_angle = 0;
	goal_distance = 0;
	goal_reached_timer = 0;

	resetEncoders();
	resetMotors();

	setState(REST);

}

void accelerateLeft() {

	float derivative = left_error - old_left_error; // ticks per ms

	test1 = derivative;
	if (derivative < velocity_left * 34.0)
		left_PWM_value += xaccelerationTEST;

	if (derivative > velocity_left * 34.0)
		left_PWM_value -= xaccelerationTEST;

}

void accelerateRight() {

	float derivative = right_error - old_right_error;

	test2 = derivative;
	if (derivative < velocity_right * 34.0)
		right_PWM_value += xaccelerationTEST;

	if (derivative > velocity_right * 34.0)
		right_PWM_value -= xaccelerationTEST;

}

void PDController() {

	float adjustedAngle = goal_angle + IRadjustment;

	angleError = adjustedAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);

	distanceError = goal_distance - ((getLeftEncoderCounts() + getRightEncoderCounts())/2);

	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);

	left_error = getLeftEncoderCounts();
	right_error = getRightEncoderCounts();

	if (state == MOVING && fabs(distanceError) > 100)
	{		// If we're going straight and not at the end, apply acceleration

		if (fabs(distanceCorrection - oldDistanceCorrection) > xacceleration)
			distanceCorrection = oldDistanceCorrection + (xacceleration * sign(distanceCorrection - oldDistanceCorrection));
	}

	switch(state) {		// Apply lower limits of PWM for various states
		case MOVING:
		case EXPLORING:
			if (fabs(distanceCorrection) > 0.01 && fabs(distanceCorrection) < PWM_min_x)
				distanceCorrection = sign(distanceCorrection) * PWM_min_x;
			break;
		case TURNING:
			if (fabs(angleCorrection) > 0.01 && fabs(angleCorrection) < PWM_min_w)
				angleCorrection = sign(angleCorrection) * PWM_min_w;
			break;
		default:
			break;
	}

}

void updatePID() {

///// CALCULATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

	oldAngleError = oldAngleErrors[9];
	oldDistanceError = oldDistanceErrors[9];
	old_left_error = old_left_errors[9];
	old_right_error = old_right_errors[9];

//////////	CALCULATE ANGLE CORRECTION AND DISTANCE CORRECTION	/////////////////////

	PDController();

	if (state == EXPLORING)
	{
		distanceError = 306;
		distanceCorrection = explore_speed;

		if (fabs(distanceCorrection - oldDistanceCorrection) > xacceleration)
		{
			distanceCorrection = oldDistanceCorrection + xacceleration;
		}
	}

////////// 	CALCULATE AND SET MOTOR PWM VALUES	////////////////////////////

	if (fabs(distanceCorrection) > PWM_max_x)		// Upper Limit for PWM
		distanceCorrection = sign(distanceCorrection) * PWM_max_x;

	if (fabs(angleCorrection) > PWM_max_w)
		angleCorrection = sign(angleCorrection) * PWM_max_w;

	if (state == ACCELERATING)
	{
		accelerateLeft();
		accelerateRight();
	}

	else
	{
		left_PWM_value = (distanceCorrection + angleCorrection);
		right_PWM_value = (distanceCorrection - angleCorrection);
	}


	// Apply lower PWM limits for small adjustments
	if (state == REST || state == ACCELERATING || fabs(distanceError) < 60 || fabs (angleError) < 60)
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWM_min)
		{
			left_PWM_value = sign(left_PWM_value) * PWM_min;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWM_min)
		{
			right_PWM_value = sign(right_PWM_value) * PWM_min;
		}
	}
	else	// If under PWM limits, normalize values
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWM_min)
		{
			right_PWM_value = right_PWM_value - (sign(right_PWM_value) * (PWM_min - fabs(left_PWM_value)));
			left_PWM_value = sign(left_PWM_value) * PWM_min;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWM_min)
		{
			left_PWM_value = left_PWM_value - (sign(left_PWM_value) * (PWM_min - fabs(right_PWM_value)));
			right_PWM_value = sign(right_PWM_value) * PWM_min;
		}
	}

	if (fabs(left_PWM_value) > PWM_max_x)		// Upper Limit for PWM
		left_PWM_value = sign(left_PWM_value) * PWM_max_x;

	if (fabs(right_PWM_value) > PWM_max_x)
		right_PWM_value = sign(right_PWM_value) * PWM_max_x;

	setMotorLPWM(left_PWM_value);
	setMotorRPWM(right_PWM_value);

	if(angleError < 30 && angleError > -30 && distanceError < 30 && distanceError > -30)
		goal_reached_timer++;					// Increments goal reached timer when errors are within a certain threshold

	else
		goal_reached_timer = 0;


	oldDistanceCorrection = distanceCorrection;

	test1 = left_error - old_left_error;

	for(int i = 9; i > 0; i--)
		oldAngleErrors[i] = oldAngleErrors[i-1];	// Adds the newest angleError to array and shifts everything to the right
	oldAngleErrors[0] = angleError;

	for(int i = 9; i > 0; i--)
		oldDistanceErrors[i] = oldDistanceErrors[i-1];	// Adds the newest distanceError to array and shifts everything right
	oldDistanceErrors[0] = distanceError;

	for(int i = 9; i > 0; i--)
		old_left_errors[i] = old_left_errors[i-1];
	old_left_errors[0] = left_error;

	for(int i = 9; i > 0; i--)
		old_right_errors[i] = old_right_errors[i-1];
	old_right_errors[0] = right_error;

}

int8_t PIDdone(){ // There is no bool type in C. True/False values are represented as 1 or 0.

	if (goal_reached_timer >= 50)
	{
		resetPID();
		setState(REST);
		return 1;
	}
	else
		return 0;

}
