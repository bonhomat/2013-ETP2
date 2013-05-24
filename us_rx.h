/**************************************************************************//**
 * @file us_rx.h
 *
 * @brief RX functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-30
 * @version 0.1
 *
 * @verbatim
 * Ultra sonic receiver with ADC, defines, configuration and DMA functions
 * @endverbatim
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/



#ifndef __us_rx_H
#define __us_rx_H

/**************************************************************************//**
 * @brief Resets the DMA to reset values for other use
 *
 *****************************************************************************/
extern void DMA_Reset(void);

/**************************************************************************//**
 * @brief Initializes all needed ADC channel
 *
 *****************************************************************************/
extern void InitADC(void);

/**************************************************************************//**
 * @brief Start Measuring routine
 *
 *****************************************************************************/
extern void RX_Measure(void);

#endif