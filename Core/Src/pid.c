/*
 * pid.c
 */

#include <math.h>
#include "pid.h"
#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "irs.h"

// Goal Parameters
float goalDistance = 0;
float goalAngle = 0;
int goalLeftIR = 0;
int goalRightIR = 0;
int IRAngleOffset = 0;

// Error Parameters
float angleError = 0;
float oldAngleError = 0;
float oldAngleErrors[5] = {0};
float angleCorrection = 0;
float derivativeAngleCorrection = 0;
float proportionalAngleCorrection = 0;

float distanceError = 0;
float oldDistanceError = 0;
float oldDistanceErrors[5] = {0};
float distanceCorrection = 0;
float oldDistanceCorrection = 0;

float IRadjustment = 0;

// Constants
const float kPw = 0.001;	// 0.01
const float kDw = 0.000;	// 0.0002
const float kPx = 0.001;	// 0.001
const float kDx = 0.0000;		//0.0

float kPir = 0.0203;
float kPir2 = 0.0103;

const float xacceleration = 0.001; // 0.002

const float PWMMaxx = 0.65;
const float PWMMaxw = 0.4;
const float PWMMinx = 0.3;
const float PWMMinw = 0.3;
const float PWMMin = 0.25;

const float explore_speed = 0.4;

// Miscellaneous
STATE state = REST;
float left_PWM_value = 0;
float right_PWM_value = 0;
int goal_reached_timer = 0;

void setPIDGoalD(int16_t distance) { goalDistance = distance; }
void setPIDGoalA(int16_t angle) { goalAngle = angle; }
void setState(STATE curr_state) { state = curr_state; }
void setIRAngleOffset(int16_t angleOffset, int16_t leftGoal, int16_t rightGoal) {

	IRAngleOffset = angleOffset;
	goalLeftIR = leftGoal;
	goalRightIR = rightGoal;

}

// TODO: CHANGE TO USE WALL CHECK FUNCTIONS
void setIRAngle(float left, float right){

	if (left > 600 && right > 600 && goalAngle == 0)
	{
		IRadjustment = (kPir * ((left - right) - IRAngleOffset));
	}
	else if (left > 600 && goalAngle == 0)
	{
		IRadjustment = (kPir2 * (left - goalLeftIR));
	}
	else if (right > 600 && goalAngle == 0)
	{
		IRadjustment = (kPir2 * (goalRightIR - right));
	}
	else
		IRadjustment = 0;
}

void resetPID() {

//////////////	RESET ALL ANGLE AND DISTANCE ERRORS TO 0	////////////////
	angleError = 0;
	oldAngleError = 0;
	angleCorrection = 0;

	for (int k = 0; k < 5; k++)
		oldAngleErrors[k] = 0;

	distanceError = 0;
	oldDistanceError = 0;
	distanceCorrection = 0;

	for (int i = 0; i < 5; i++)
		oldDistanceErrors[i] = 0;

	IRadjustment = 0;

////////////// 	RESET ALL GOALS AND ENCODER COUNTS TO 0 	///////////////////
	goalAngle = 0;
	goalDistance = 0;
	goal_reached_timer = 0;

	resetEncoders();
	resetMotors();

}

void updatePID() {

///// CALCULATE AVERAGE OF PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////
	float angleErrorTotal;
	float distanceErrorTotal;

	for(int k = 0; k < 5; k++)
	{
		angleErrorTotal += oldAngleErrors[k];		// Finds the total for the previous 10 error values
	}

	oldAngleError = angleErrorTotal/5.0;				// oldAngleError = Average of previous 10 error values

	for(int i = 0; i < 5; i++)
	{
		distanceErrorTotal += oldDistanceErrors[i];		// Finds the total for the previous 10 error values
	}

	oldDistanceError = distanceErrorTotal/5;			// oldDistanceError = Average of previous 10 error values

//////////	CALCULATE ANGLE CORRECTION AND DISTANCE CORRECTION	/////////////////////

	float adjustedAngle = goalAngle + IRadjustment;

	angleError = adjustedAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);

	distanceError = goalDistance - ((getLeftEncoderCounts() + getRightEncoderCounts())/2);

	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);

	if (state == EXPLORING)
	{
		distanceCorrection = explore_speed;
	}

	for(int k = 4; k > 0; k--)
		oldAngleErrors[k] = oldAngleErrors[k-1];	// Adds the newest angleError to array and shifts everything to the right
	oldAngleErrors[0] = angleError;

	for(int i = 4; i > 0; i--)
		oldDistanceErrors[i] = oldDistanceErrors[i-1];	// Adds the newest distanceError to array and shifts everything right
	oldDistanceErrors[0] = distanceError;

////////// 	CALCULATE AND SET MOTOR PWM VALUES	////////////////////////////

	if (fabs(distanceCorrection) > PWMMaxx)		// Upper Limit for PWM
		distanceCorrection = sign(distanceCorrection) * PWMMaxx;

	if (fabs(angleCorrection) > PWMMaxw)
		angleCorrection = sign(angleCorrection) * PWMMaxw;

	if (state == MOVING && fabs(distanceError) > 100)
	{		// If we're going straight and not at the end, apply acceleration

		if (fabs(distanceCorrection - oldDistanceCorrection) > xacceleration)
					distanceCorrection = oldDistanceCorrection + (xacceleration *
							(distanceCorrection - oldDistanceCorrection)/fabs(distanceCorrection - oldDistanceCorrection));
	}

	if (state == EXPLORING)
	{
		if (fabs(distanceCorrection - oldDistanceCorrection) > xacceleration)
		{
			distanceCorrection = oldDistanceCorrection + xacceleration;
		}
	}

	switch(state) {		// Apply lower limits of PWM for various states
		case MOVING:
			if (fabs(distanceCorrection) > 0.01 && fabs(distanceCorrection) < PWMMinx)
				distanceCorrection = sign(distanceCorrection) * PWMMinx;
			break;

		case TURNING:
			if (fabs(angleCorrection) > 0.01 && fabs(angleCorrection) < PWMMinw)
				angleCorrection = sign(angleCorrection) * PWMMinw;
			break;

		case REST:
			break;
	}

	left_PWM_value = (distanceCorrection + angleCorrection);
	right_PWM_value = (distanceCorrection - angleCorrection);

	oldDistanceCorrection = distanceCorrection;

	// Apply lower PWM limits for small adjustments
	if (state == REST || fabs(distanceError) < 60 || fabs (angleError) < 60)
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWMMin)
		{
			left_PWM_value = sign(left_PWM_value) * PWMMin;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWMMin)
		{
			right_PWM_value = sign(right_PWM_value) * PWMMin;
		}
	}
	else	// If under PWM limits, normalize values
	{
		if (fabs(left_PWM_value) > 0.01 && fabs(left_PWM_value) < PWMMin)
		{
			right_PWM_value = right_PWM_value - (sign(right_PWM_value) * (PWMMin - fabs(left_PWM_value)));
			left_PWM_value = sign(left_PWM_value) * PWMMin;
		}

		if (fabs(right_PWM_value) > 0.01 && fabs(right_PWM_value) < PWMMin)
		{
			left_PWM_value = left_PWM_value - (sign(left_PWM_value) * (PWMMin - fabs(right_PWM_value)));
			right_PWM_value = sign(right_PWM_value) * PWMMin;
		}
	}

	setMotorLPWM(left_PWM_value);
	setMotorRPWM(right_PWM_value);

	if(angleError < 30 && angleError > -30 && distanceError < 30 && distanceError > -30)
		goal_reached_timer++;					// Increments goal reached timer when errors are within a certain threshold

	else
		goal_reached_timer = 0;

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
