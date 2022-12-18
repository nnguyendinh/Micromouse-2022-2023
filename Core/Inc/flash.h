/*
 * flash.h
 *
 *  Created on: Nov 25, 2022
 *      Author: Nathan Nguyendinh
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>

void writeFlash(int16_t horzWall[][16], int16_t vertWall[][17]);
void readFlash(int16_t horzWall[][16], int16_t vertWall[][17]);

#endif /* INC_FLASH_H_ */
