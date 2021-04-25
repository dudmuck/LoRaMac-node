#include <stdio.h>
#include "board.h"
#include "board-config.h"
#include "timer.h"
#include "gpio.h"
#include "lpm-board.h"

volatile unsigned mli;
static TimerEvent_t myTimer;

Gpio_t Led;

static void OnTimerEvent( void* context )
{
    TimerStart( &myTimer );
    GpioToggle( &Led );
    printf("mli %u\r\n", mli);
    mli = 0;
}

int main( void )
{
    BoardInitMcu( );
    BoardInitPeriph( );

    GpioInit( &Led, ON_BOARD_LED, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    TimerInit( &myTimer, OnTimerEvent );
    TimerSetValue( &myTimer, 333 );
    TimerStart( &myTimer );
    printf("\r\nstart\r\n");

    // Disables OFF mode - Enables lowest power mode (STOP)
    //LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );

    mli = 0;
    while( 1 )
    {
        mli++;
        BoardLowPowerHandler( );
    }
}

