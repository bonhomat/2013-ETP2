/**************************************************************************//**
 * @file us_uart.h
 *
 * @brief Serial interface 
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-30
 * @version 0.1
 *
 * @verbatim
 * Sending receiving data over serial interface
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __us_uart_H
#define __us_uart_H

/**************************************************************************//**
 * @brief Initializes Uart interface 
 * 
 *****************************************************************************/
extern void InitUart(void);

/**************************************************************************//**
 * @brief function for transmitting over serial interface
 * 
 *****************************************************************************/
extern void SendUart(void);


#endif