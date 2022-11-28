#include "flash.h"
#include <stdint.h>
#include "stm32f2xx.h"

int16_t mazeWalls[16][16]; // to hold wall information.
uint32_t startAddress = 0x080E0000;

void writeFlash(int16_t horzWall[][], int16_t vertWall[][])
{
	uint32_t i, j;
	HAL_FLASH_Unlock();

	/* Clear All Pending Flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

//	for(i=0; i<16; i++)
//		for(j=0; j<16; j++)
//	 	 	HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, (startAddress + (i*16+j)*4), mazeWalls[i][j]);
    for (i = 0; i < 17; i++) {
        for (j = 0; j < 16; j++) {
            HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, (startAddress + (i*16+j)*4), horzWall[i][j]);
        }
    }
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 17; j++) {
            HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, (startAddress + (16*17*4) + (i*17+j)*4), vertWall[i][j]);
        }
    }

	HAL_FLASH_Lock();
}

void readFlash(int16_t horzWall[][], int16_t vertWall[][])
{
	uint32_t i, j;
//	for(i=0; i < 16; i++)
//		for(j=0; j < 16; j++)
//			mazeWalls[i][j] = *(int16_t *)(startAddress + (i*16+j)*4);
    for (i = 0; i < 17; i++) {
        for (j = 0; j < 16; j++) {
            horzWall[i][j] = *(int16_t *)(startAddress + (i*16+j)*4);
        }
    }
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 17; j++) {
            vertWall[i][j] = *(int16_t *)(startAddress + (16*17*4) + (i*17+j)*4);
        }
    }
}
