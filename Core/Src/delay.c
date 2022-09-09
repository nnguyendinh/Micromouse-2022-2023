/*
 * delay.c
 * BASED ON DWT_DELAY LIBRARY: https://github.com/keatis/dwt_delay
 */

#include "main.h"
#include "delay.h"

void Delay_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

// Delays for us microseconds
void delayMicroseconds(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT,
    delayTicks = us * 16; // Our MCU runs at 16 MHz, so each microsecond lasts 16 clock ticks

    while (DWT->CYCCNT - startTick < delayTicks);
}
