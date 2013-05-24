/**************************************************************************//**
 * @file clock.c
 *
 * @brief clock and RTC functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-23
 * @version 0.1
 *
 * @verbatim
 * clock and RTC used defines, configuration and functions
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
/***************************************************************************//**
 * Includes
 *******************************************************************************/
#include "efm32.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_lcd.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_rtc.h"
#include "segmentlcd.h"

#include "globals.h"

#include "clock.h"

/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * Static variables
 *****************************************************************************/
uint16_t  RTC_cntr  = 0;
bool      RTC_tick  = false;




/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/

 
/**************************************************************************//**
 * @brief Initialize GPIO clock
 * 
 *****************************************************************************/
void InitGPIOClk(void)
{
 
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

}//END: InitGPIOClk

/**************************************************************************//**
 * @brief Initialize clock system
 * 
 *****************************************************************************/
void InitClk(void)
{
  /* Activate high frequency crystal oscillator HFXO */
  #define ON      true    // Enable oscillator
  #define DO_WAIT true    // return when osc is ready to use
  CMU_OscillatorEnable( cmuOsc_HFXO, ON, DO_WAIT );
  CMU_ClockSelectSet  ( cmuClock_HF, cmuSelect_HFXO );

}// End: InitClk


/**************************************************************************//**
 * @brief starts Oscillator and routing for RTC
 * 
 *****************************************************************************/
void startLFXOForRTC(void)
{
  /* Start LFXO and wait until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Select LFXO as the clock source for RTC clock domain */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
  
  /* Enable clock to RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enable clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
}


/**************************************************************************//**
 * @brief Initialize RTC
 * 
 *****************************************************************************/
void InitRTC(void) 
{
  /* Enable clock for RTC */
  startLFXOForRTC();
  
  #define RTC_FREQ 32768
  
  /* Configure to overflow every 50ms */
  RTC_CompareSet(0, 50 * RTC_FREQ / 1000);
  
  /* Configure and enable RTC */
  RTC_Init_TypeDef rtcInit;
  rtcInit.comp0Top = true;
  rtcInit.debugRun = false;
  rtcInit.enable   = true;
  RTC_Init(&rtcInit);

  /* Enable overflow interrupt */
  RTC_IntEnable(RTC_IEN_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}



/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief Initializes all needed clocks
 * 
 *****************************************************************************/
void InitClocks(void)
{
  InitClk();
  InitGPIOClk();
  InitRTC();
}



/******************************************************************************
 * @brief RTC_IRQHandler on Compare 0 Top counting Interrupts
 *
 *****************************************************************************/
void RTC_IRQHandler(void) 
{
  /* Get and clear interrupt flags */
  uint32_t flags = RTC_IntGet();
  RTC_IntClear(flags);
  
  /* Toggle LED if overflow flag was set */
  if ( flags & RTC_IF_COMP0 ) {
    RTC_cntr++;
    RTC_tick = true;
  }
}



