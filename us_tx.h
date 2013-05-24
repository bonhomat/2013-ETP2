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


/*******************************************************************************
 * @brief Load TIMER1 registers for continous signal
 ******************************************************************************/
extern void TX_Start_Signal(void);

/*******************************************************************************
 * @brief Load TIMER1 registers to stop continous signal
 ******************************************************************************/
extern void TX_Stop_Signal(void);


/*******************************************************************************
 * @brief Load TIMER1 registers for a Burst-Run
 ******************************************************************************/
extern void TX_Start_Burst(void);

/*******************************************************************************
 * @brief Load TIMER1 registers for stopping Burst-Run
 ******************************************************************************/
extern void TX_Stop_Burst(void);


/*******************************************************************************
 * @brief Put TIMER1 into running mode
 ******************************************************************************/
extern void TX_Timer_Run(void);

/*******************************************************************************
 * @brief Stop TIMER1
 ******************************************************************************/
extern void TX_Timer_Stop(void);


/******************************************************************************
 * @brief TIMER1_IRQHandler on active burst/rburst counting Interrupts of CC2
 *
 *****************************************************************************/
extern void TIMER1_IRQHandler(void);


/*******************************************************************************
 * @brief Initialize TIMER1 in Up/Down Count mode with interrupts on overflow
 *
 ******************************************************************************/
extern void InitTimer1(void);


#endif

