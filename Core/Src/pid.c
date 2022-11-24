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

// Constants
const float kPw = 0.003;	// 0.003
const float kDw = 0.0005;	// 0.0005
const float kPx = 0.001;	// 0.001
const float kDx = 0.0000;		//0.0

const float front_kPx = 0.6;
const float front_kPw = 0.35;

const float kPir = 0.050;		// 0.05	for 2 walls
const float kPir2 = 0.075;		// 0.025 for 1 wall

const float xacceleration = 0.001; // 0.001

const float PWMMaxx = 0.5; // 0.65
const float PWMMaxw = 0.35;	//0.35
const float PWMMinx = 0.32;	// 0.32
const float PWMMinw = 0.32;	// 0.32
const float PWMMin = 0.28;	// 0.28

const float explore_speed = 0.4;

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

float left_distance = 0;
float old_left_distance = 0;
float old_left_distances[10] = {0};

float right_distance = 0;
float old_right_distance = 0;
float old_right_distances[10] = {0};

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

	setPIDGoalA(FRONT_kPw * ((goal_forward_left - curr_forward_left) - (goal_forward_right - curr_forward_right)));
	setPIDGoalD(FRONT_kPx * ((goal_forward_left - curr_forward_left + goal_forward_right - curr_forward_left)/2));

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

float accelerateLeftPWM() {

	float derivative = left_distance - old_left_distance; // ticks per ms

	test1 = derivative;
	if (derivative < velocity_left * 34.0)
		return left_PWM_value + ACC_CONSTANT;

	if (derivative > velocity_left * 34.0)
		return left_PWM_value - ACC_CONSTANT;

	return left_PWM_value;

}

float accelerateRightPWM() {

	float derivative = right_distance - old_right_distance;

	test2 = derivative;
	if (derivative < velocity_right * 34.0)
		return right_PWM_value + ACC_CONSTANT;

	if (derivative > velocity_right * 34.0)
		return right_PWM_value - ACC_CONSTANT;

	return right_PWM_value;

}

void PDController() {

//////////////////////////	CALCULATE DISTANCE AND ANGLE CORRECTION /////////////////////////

	float adjustedAngle = goal_angle + IRadjustment;

	angleError = adjustedAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);

	distanceError = goal_distance - ((getLeftEncoderCounts() + getRightEncoderCounts())/2);

	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);

	left_distance = getLeftEncoderCounts();
	right_distance = getRightEncoderCounts();

	if (state == MOVING && fabs(distanceError) > 100)
	{		// If we're going straight and not at the end, apply acceleration
		if (fabs(distanceCorrection - oldDistanceCorrection) > xacceleration)
		{
			distanceCorrection = oldDistanceCorrection + (xacceleration * sign(distanceCorrection - oldDistanceCorrection));
		}
	}

////////////////////// ROUND DISTANCE OR ANGLE CORRECTION	//////////////////////////

	switch(state) {		// Apply lower limits of PWM for various states
		case EXPLORING:
			distanceCorrection = (accelerateLeftPWM() + accelerateRightPWM())/2;
		case MOVING:
			if (fabs(distanceCorrection) > 0.03 && fabs(distanceCorrection) < PWMMinx)
				distanceCorrection = sign(distanceCorrection) * PWMMinx;
			break;
		case TURNING:
			if (fabs(angleCorrection) > 0.01 && fabs(angleCorrection) < PWM_MIN_W)
				angleCorrection = sign(angleCorrection) * PWM_MIN_W;
			break;
		case CURVING:
			if (fabs(distanceError) < 60)
				setState(REST);
		default:
			break;
	}

	if (fabs(distanceCorrection) > PWM_MAX_X)		// Upper Limit for PWM
		distanceCorrection = sign(distanceCorrection) * PWM_MAX_X;

	if (fabs(angleCorrection) > PWM_MAX_W)
		angleCorrection = sign(angleCorrection) * PWM_MAX_W;

	if (state == ACCELERATING || state == CURVING)
	{
		left_PWM_value = accelerateLeftPWM();
		right_PWM_value = accelerateRightPWM();
		return;
	}

	left_PWM_value = (distanceCorrection + angleCorrection);
	right_PWM_value = (distanceCorrection - angleCorrection);

}

void updatePID() {

///// CALCULATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

	oldAngleError = oldAngleErrors[9];
	oldDistanceError = oldDistanceErrors[9];
	old_left_distance = old_left_distances[9];
	old_right_distance = old_right_distances[9];

//////////////////////	CALCULATE MOTOR PWM VALUES	/////////////////////

	PDController();

////////////////////	NORMALIZE LEFT AND RIGHT PWM VALUES ////////////////

	// Apply lower PWM limits for small adjustments
	if (state == REST || state == ACCELERATING || fabs(distanceError) < 60 || fabs (angleError) < 60)
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWM_MIN)
		{
			left_PWM_value = sign(left_PWM_value) * PWM_MIN;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWM_MIN)
		{
			right_PWM_value = sign(right_PWM_value) * PWM_MIN;
		}
	}
	else	// If under PWM limits, normalize values
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWM_MIN)
		{
			right_PWM_value = right_PWM_value - (sign(right_PWM_value) * (PWM_MIN - fabs(left_PWM_value)));
			left_PWM_value = sign(left_PWM_value) * PWM_MIN;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWM_MIN)
		{
			left_PWM_value = left_PWM_value - (sign(left_PWM_value) * (PWM_MIN - fabs(right_PWM_value)));
			right_PWM_value = sign(right_PWM_value) * PWM_MIN;
		}
	}

	if (fabs(left_PWM_value) > PWM_MAX)
	{
		left_PWM_value = sign(left_PWM_value) * PWM_MAX;
	}

	if (fabs(right_PWM_value) > PWM_MAX)
	{
		right_PWM_value = sign(right_PWM_value) * PWM_MAX;
	}



//////////////////	SET PWM VALUES AND CHECK FOR GOAL REACHED ////////////////////////

	setMotorLPWM(left_PWM_value);
	setMotorRPWM(right_PWM_value);

	if(angleError < 30 && angleError > -30 && distanceError < 30 && distanceError > -30)
		goal_reached_timer++;					// Increments goal reached timer when errors are within a certain threshold

	else
		goal_reached_timer = 0;

///////////////////// UPDATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

	oldDistanceCorrection = distanceCorrection;

	for(int i = 9; i > 0; i--)
		oldAngleErrors[i] = oldAngleErrors[i-1];	// Adds the newest angleError to array and shifts everything to the right
	oldAngleErrors[0] = angleError;

	for(int i = 9; i > 0; i--)
		oldDistanceErrors[i] = oldDistanceErrors[i-1];	// Adds the newest distanceError to array and shifts everything right
	oldDistanceErrors[0] = distanceError;

	for(int i = 9; i > 0; i--)
		old_left_distances[i] = old_left_distances[i-1];
	old_left_distances[0] = left_distance;

	for(int i = 9; i > 0; i--)
		old_right_distances[i] = old_right_distances[i-1];
	old_right_distances[0] = right_distance;

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

	left_distance = 0;
	old_left_distance = 0;
	for (int i = 0; i < 10; i++)
		old_left_distances[i] = 0;

	right_distance = 0;
	old_right_distance = 0;
	for (int i = 0; i < 10; i++)
		old_right_distances[i] = 0;

	IRadjustment = 0;

////////////// 	RESET ALL GOALS AND ENCODER COUNTS TO 0 	///////////////////
	goal_angle = 0;
	goal_distance = 0;
	goal_reached_timer = 0;

	resetEncoders();
	resetMotors();

	setState(REST);

}
