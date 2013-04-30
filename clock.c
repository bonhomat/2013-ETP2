
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
#include "segmentlcd.h"
//#include "em_chip.h"

#include "globals.h"


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
  #define ON      true                      	// Enable oscillator
  #define DO_WAIT true                      	// return when osc is ready to use
  CMU_OscillatorEnable( cmuOsc_HFXO, ON, DO_WAIT );
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );

}// End: InitClk


/**************************************************************************//**
 * @brief Initializes all needed clocks
 * 
 *****************************************************************************/
void InitClocks(void)
{
  InitClk();
  InitGPIOClk();
}