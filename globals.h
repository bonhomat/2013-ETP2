/**************************************************************************//**
 * @file globals.h
 *
 * @brief global defines, structs and variables
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-16
 * @version 0.1
 *
 * @verbatim
 * Global used defines, structs and variables used in the project.
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __globals_H
#define __globals_H

/*******************************************************************************
 **************************   DATA DEFINITIONS *********************************
 ******************************************************************************/

/***************************************************************************//**
 * Global defines
 *******************************************************************************/
/* Drive port definitions */
#define DL_PORT     gpioPortD   /**< em_lib name of port used for DL */
#define DH_PORT     gpioPortD   /**< em_lib name of port used for DH */
#define DL_PIN      6           /**< Bit/Pin # for DL in above port */
#define DH_PIN      7           /**< Bit/Pin # for DH in above port */
#define DL_OFF_LVL  0           /**< Logic level when DL is off */
#define DH_OFF_LVL  1           /**< Logic level when DH is off */
                                /**< em_lib functions for setup DH CC Stop */
#define DH_CC_STOP  ( TIMER_CC_CTRL_ICEDGE_BOTH         | \
                      TIMER_CC_CTRL_CUFOA_SET           | \
                      TIMER_CC_CTRL_COFOA_SET           | \
                      TIMER_CC_CTRL_CMOA_SET            | \
                      TIMER_CC_CTRL_COIST               | \
                      TIMER_CC_CTRL_MODE_OUTPUTCOMPARE)
                                /**< em_lib functions for setup DL CC Stop */
#define DL_CC_STOP  ( TIMER_CC_CTRL_ICEDGE_BOTH         | \
                      TIMER_CC_CTRL_CUFOA_CLEAR         | \
                      TIMER_CC_CTRL_COFOA_CLEAR         | \
                      TIMER_CC_CTRL_CMOA_CLEAR          | \
                      TIMER_CC_CTRL_MODE_OUTPUTCOMPARE)
                                /**< em_lib functions for setup CC Run (both) */
#define DR_CC_RUN   ( TIMER_CC_CTRL_ICEDGE_BOTH         | \
                      TIMER_CC_CTRL_CUFOA_NONE          | \
                      TIMER_CC_CTRL_COFOA_NONE          | \
                      TIMER_CC_CTRL_CMOA_TOGGLE         | \
                      TIMER_CC_CTRL_MODE_OUTPUTCOMPARE)

#define CC2_RUN       TIMER_CC_CTRL_MODE_OUTPUTCOMPARE
#define CC2_STOP      TIMER_CC_CTRL_MODE_OFF
                      
                      
                                /**< Button port definitions */
#define PB0_PORT   gpioPortD                          //Port D
#define PB0_PIN    8                                  //Button 0 on Board tg
#define PB1_PORT   gpioPortB                          //Port B
#define PB1_PIN    11                                 //Button 1 on Board tg


                                /**< Timing definitions */
#define F_HFXO          32000000                      // Crystal oszillator frequency
#define F_TX            40000                         // 40kHz --> TX frequency


                                /**< Counter definitions */
#define BURST_PULSE_CNT   80                          // Pulsewith of burst
#define PERIOD_PULSE_CNT  80000                       // total count in one burst cycle
#define TIMER1_LOAD_VAL   201                         // the value to loat in Counter
                                                      //  of Timer1 for next bust

/*******************************************************************************
 * Enums
 *******************************************************************************/

/*******************************************************************************
* @brief Definition of Menue state phases
********************************************************************************/
typedef enum states
{
  waiting,                      /**< Waiting */
  continious,                   /**< State ON */
  sburst,                       /**< State single Burst */
  rburst,                       /**< State continious burst*/
  init                          /**< Initial state*/
} menue_state ;


/*****************************************************************************
 * global variable references
 *****************************************************************************/
extern menue_state      GUIState;
extern bool             wakeUp;
extern uint32_t         counter;
extern bool             PB1waspressed;
extern bool             GUIstatechanged;
extern bool             routineactive;


#endif

