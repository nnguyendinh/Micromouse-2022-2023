/*
 * controller.h
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

#define move_counts 612 // 612
#define turn_counts 456 // 456
#define init_counts 300

void move(int8_t n);
void turn(int8_t n);
void moveEncoderCount(int8_t n);
void explore(void);
void frontCorrection(void);

#endif /* INC_CONTROLLER_H_ */
