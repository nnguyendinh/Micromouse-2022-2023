/*
 * encoders.c
 */

#include "main.h"
#include "encoders.h"

int16_t getRightEncoderCounts() {
	return (-1*(int16_t) TIM3->CNT);
}

int16_t getLeftEncoderCounts() {
	return (-1*(int16_t) TIM8->CNT);
}

void resetEncoders() {
	TIM3->CNT = (int16_t) 0;
	TIM8->CNT = (int16_t) 0;
}
