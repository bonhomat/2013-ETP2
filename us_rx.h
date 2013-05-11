/**************************************************************************//**
 * @file us_rx.h
 *
 * @brief rx functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-30
 * @version 0.1
 *
 * @verbatim
 * us receiver with ADC, defines, configuration and DMA functions
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __us_rx_H
#define __us_rx_H

/**************************************************************************//**
 * @brief Initializes all needed ADC channel
 * 
 *****************************************************************************/
extern void InitADC(void);

/**************************************************************************//**
 * @brief Initializes all needed DMA channel
 * 
 *****************************************************************************/
extern void Measure(void);

#endif