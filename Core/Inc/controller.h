/*
 * controller.h
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

#define MOVE_COUNTS 615 // 612
#define TURN_COUNTS 459 // 458	// 442 // changed from 435 on cloth table
#define INIT_COUNTS 300

void move(int8_t n);
void turn(int8_t n);
void moveEncoderCount(int8_t n);
void explore(void);
void frontCorrection(void);
void curve(int8_t n);

#endif /* INC_CONTROLLER_H_ */
