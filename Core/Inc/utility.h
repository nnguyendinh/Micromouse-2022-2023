/*
 * utility.h
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

#include "main.h"

void setLeftWall(int wall);
void setRightWall(int wall);
void setFrontWall(int wall);

int16_t leftWallCheck(void);
int16_t rightWallCheck(void);
int16_t frontWallCheck(void);

int16_t sign(float x);

#endif /* INC_UTILITY_H_ */
