/*!
 * \file      sysIrqHandlers.c
 *
 * \brief     Default IRQ handlers
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdint.h>
#include "stm32f4xx.h"
//#include "eeprom_emul.h"

/*!
 * \brief  This function handles NMI exception.
 * \param  None
 * \retval None
 */
void NMI_Handler( void )
{
    while( 1 )
    {
    }
}

/*!
 * \brief  This function handles Hard Fault exception.
 * \param  None
 * \retval None
 */
void HardFault_Handler( void )
{
    // Go to infinite loop when Hard Fault exception occurs
    while( 1 )
    {
    }
}

/*!
 * \brief  This function handles Memory Manage exception.
 * \param  None
 * \retval None
 */
void MemManage_Handler( void )
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while ( 1 )
    {
    }
}

/*!
 * \brief  This function handles Bus Fault exception.
 * \param  None
 * \retval None
 */
void BusFault_Handler( void )
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while ( 1 )
    {
    }
}

/*!
 * \brief  This function handles Usage Fault exception.
 * \param  None
 * \retval None
 */
void UsageFault_Handler( void )
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while ( 1 )
    {
    }
}

/*!
 * \brief  This function handles Debug Monitor exception.
 * \param  None
 * \retval None
 */
void DebugMon_Handler( void )
{
}

/*!
 * \brief  This function handles Flash interrupt request.
 * \param  None
 * \retval None
 *
void FLASH_IRQHandler( void )
{
    HAL_FLASH_IRQHandler();
}*/

/*!
 * \brief  This function handles PVD interrupt request.
 * \param  None
 * \retval None
 */
void PVD_PVM_IRQHandler( void )
{
    // Loop inside the handler to prevent the Cortex from using the Flash,
    // allowing the flash interface to finish any ongoing transfer.
    while( __HAL_PWR_GET_FLAG( PWR_FLAG_PVDO ) != RESET )
    {
    }
}

