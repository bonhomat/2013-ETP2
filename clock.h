/**************************************************************************//**
 * @file clock.h
 *
 * @brief clock functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-23
 * @version 0.1
 *
 * @verbatim
 * clock used defines, configuration and functions
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __clock_H
#define __clock_H

/**************************************************************************//**
 * @brief Initializes all needed clocks
 * 
 *****************************************************************************/
extern void InitClocks(void);


/******************************************************************************
 * @brief RTC_IRQHandler on Compare 0 Top counting Interrupts
 *
 *****************************************************************************/
extern void RTC_IRQHandler(void);


#endif
