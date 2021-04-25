#include <stdio.h>
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"

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

#define RX_TIMEOUT_VALUE                            0   /* forever RX */

static RadioEvents_t RadioEvents;

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    printf("%ddBm %ddB ", rssi, snr);
    for (unsigned n = 0; n < size; n++)
        printf("%02x ", payload[n]);
    printf("\r\n");

    // no sleep, keep on receiving Radio.Sleep( );
}

void OnTxDone( void )
{
    Radio.Sleep( );
    printf("txDone\r\n");
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
}

void OnRxTimeout( void )
{
    printf("rxTimeout\r\n");
    Radio.Sleep( );
}

void OnRxError( void )
{
    Radio.Sleep( );
}

int main( void )
{
    unsigned mli = 0;
    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

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

    printf("\r\nstart rx\r\n");

    Radio.Rx( RX_TIMEOUT_VALUE );

    while( 1 )
    {
        mli++;  /* LpmEnterOffMode(), LpmEnterStopMode(), UsbIsConnected == true */

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    }
}

