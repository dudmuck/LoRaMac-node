#include "stm32l0xx.h"
#include "delay-board.h"

void DelayMsMcu( uint32_t ms )
{
    HAL_Delay( ms );
}

