#include "stm32g0xx.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "spi-board.h"

static SPI_HandleTypeDef SpiHandle[2];

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    CRITICAL_SECTION_BEGIN( );

    obj->SpiId = spiId;

    if( spiId == SPI_1 )
    {
        __HAL_RCC_SPI1_FORCE_RESET( );
        __HAL_RCC_SPI1_RELEASE_RESET( );
        __HAL_RCC_SPI1_CLK_ENABLE( );

        SpiHandle[spiId].Instance = ( SPI_TypeDef* )SPI1_BASE;

        GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI1 );
        GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI1 );
        GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI1 );
        GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF0_SPI1 );
    }
    else
    {
        __HAL_RCC_SPI2_FORCE_RESET( );
        __HAL_RCC_SPI2_RELEASE_RESET( );
        __HAL_RCC_SPI2_CLK_ENABLE( );

        SpiHandle[spiId].Instance = ( SPI_TypeDef* )SPI2_BASE;

        GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI2 );
        GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI2 );
        GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF0_SPI2 );
        GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF0_SPI2 );
    }

    if( nss == NC )
    {
        SpiHandle[spiId].Init.NSS = SPI_NSS_SOFT;
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 0 );
    }
    else
    {
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 1 );
    }
    //SpiFrequency( obj, 10000000 );
    SpiFrequency( obj, 1000000 );

    HAL_SPI_Init( &SpiHandle[spiId] );

    CRITICAL_SECTION_END( );
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    HAL_StatusTypeDef errcode;
    uint8_t rxData = 0;
    uint8_t txData = outData;

#if 0
    if( ( obj == NULL ) || ( SpiHandle[obj->SpiId].Instance ) == NULL )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    __HAL_SPI_ENABLE( &SpiHandle[obj->SpiId] );

    CRITICAL_SECTION_BEGIN( );

    while( __HAL_SPI_GET_FLAG( &SpiHandle[obj->SpiId], SPI_FLAG_TXE ) == RESET );
    SpiHandle[obj->SpiId].Instance->DR = ( uint16_t ) ( outData & 0xFF );

    while( __HAL_SPI_GET_FLAG( &SpiHandle[obj->SpiId], SPI_FLAG_RXNE ) == RESET );
    rxData = ( uint16_t ) SpiHandle[obj->SpiId].Instance->DR;

    CRITICAL_SECTION_END( );
#endif /* if 0 */
//HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
    errcode = HAL_SPI_TransmitReceive(&SpiHandle[obj->SpiId], &txData, &rxData, 1, 5000);
    if (errcode != HAL_OK) {
        for (;;)
            asm("nop");
    }
    return( rxData );
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    uint32_t divisor = 0;
    uint32_t sysClkTmp = SystemCoreClock;
    uint32_t baudRate;

    while( sysClkTmp > hz )
    {
        divisor++;
        sysClkTmp = ( sysClkTmp >> 1 );

        if( divisor >= 7 )
        {
            break;
        }
    }

    baudRate =( ( ( divisor & 0x4 ) == 0 ) ? 0x0 : SPI_CR1_BR_2 ) |
              ( ( ( divisor & 0x2 ) == 0 ) ? 0x0 : SPI_CR1_BR_1 ) |
              ( ( ( divisor & 0x1 ) == 0 ) ? 0x0 : SPI_CR1_BR_0 );

    SpiHandle[obj->SpiId].Init.BaudRatePrescaler = baudRate;
}

void SpiFormat( Spi_t *obj, uint32_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    SpiHandle[obj->SpiId].Init.Direction = SPI_DIRECTION_2LINES;
    if( bits == SPI_DATASIZE_8BIT )
    {
        SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_8BIT;
    }
    else
    {
        SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_16BIT;
    }
    SpiHandle[obj->SpiId].Init.CLKPolarity = cpol;
    SpiHandle[obj->SpiId].Init.CLKPhase = cpha;
    SpiHandle[obj->SpiId].Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle[obj->SpiId].Init.TIMode = SPI_TIMODE_DISABLE;
    SpiHandle[obj->SpiId].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle[obj->SpiId].Init.CRCPolynomial = 7;
    SpiHandle[obj->SpiId].Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    SpiHandle[obj->SpiId].Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if( slave == 0 )
    {
        SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_MASTER;
    }
    else
    {
        SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_SLAVE;
    }
}

void SpiDeInit( Spi_t *obj )
{
    HAL_SPI_DeInit( &SpiHandle[obj->SpiId] );

    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}
