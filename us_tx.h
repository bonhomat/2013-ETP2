/**************************************************************************//**
 * @file us_tx.h
 *
 * @brief us transmitter functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-23
 * @version 0.1
 *
 * @verbatim
 * us transmission with timer, defines, configuration and timer-functions
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __us_tx_H
#define __us_tx_H


/******************************************************************************
 * @brief TIMER0_IRQHandler on active burst/rburst counting Interrupts of CC2
 *
 *****************************************************************************/
void TIMER0_IRQHandler(void)


/*******************************************************************************
 * @brief Initialize TIMER0 in Up/Down Count mode with interrupts on overflow
 *
 ******************************************************************************/
static void InitTimer0(void)

#endif