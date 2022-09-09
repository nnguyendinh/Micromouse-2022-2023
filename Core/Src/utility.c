/*
 * utility.c
 */

#include "utility.h"

setLeftWall(int wall) { left_wall_threshold = wall; }
setRightWall(int wall) { right_wall_threshold = wall; }
setFrontWall(int wall) { front_wall_threshold = wall; }

int16_t leftWallCheck() {
	if (readIR(IR_LEFT) > left_wall_threshold) {
		left_wall = 1;
	}
	else {
		left_wall = 0;
	}
	return left_wall;
}

int16_t rightWallCheck() {
	if (readIR(IR_RIGHT) > right_wall_threshold) {
		right_wall = 1;
	}
	else {
		right_wall = 0;
	}
	return right_wall;
}

int16_t frontWallCheck() {
	if (readIR(IR_FRONT_LEFT) > front_wall_threshold) {
		front_wall = 1;
	}
	else {
		front_wall = 0;
	}
	return front_wall;
}

int16_t sign(float x) {
	if (x > 0)
	{
		return 1;
	}
	else if (x < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
