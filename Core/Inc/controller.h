/*
 * controller.h
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

void move(int8_t n);
void turn(int8_t n);
void moveEncoderCount(int8_t n);

const int move_counts = 612; // 612
const int turn_counts = 456; // 456

#endif /* INC_CONTROLLER_H_ */
