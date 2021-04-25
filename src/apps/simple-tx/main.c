#include <stdio.h>
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "lpm-board.h"

#define BUFFER_SIZE                                 64 // Define the payload size here

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
                                                              //
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define RF_FREQUENCY                                915000000 // Hz

static RadioEvents_t RadioEvents;
static TimerEvent_t TxTimer;
bool radio_to_sleep;
volatile unsigned mli;

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    radio_to_sleep = true;
}

void OnTxDone( void )
{
    radio_to_sleep = true;
    TimerStart( &TxTimer );
    printf("txDone %u\r\n", mli);
    mli = 0;
}

void OnTxTimeout( void )
{
    //radio_to_sleep = true;
    printf("OnTxTimeout\r\n");
}

void OnRxTimeout( void )
{
    radio_to_sleep = true;
}

void OnRxError( void )
{
    radio_to_sleep = true;
}

static void OnTxTimerEvent( void* context )
{
    uint8_t buf[8];
    static uint8_t cnt = 0;

    printf("tx cnt %u\r\n", cnt++);

    for (unsigned i = 0; i < sizeof(buf); i++)
        buf[i] = cnt;

    Radio.Send(buf, sizeof(buf));
}

int main( void )
{
    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );

    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer, 500 );
    TimerStart( &TxTimer );

    printf("\r\nstart\r\n");
    mli = 0;

    while( 1 )
    {
        mli++;

        if (radio_to_sleep) {
            Radio.Sleep( );
            radio_to_sleep = false;
        }

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    }
}
