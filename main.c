/**************************************************************************//**
 * @file main.c
 *
 * @brief Main file of the receiver project
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-16
 * @version 0.1
 *
 * @verbatim
 * Main file composes all parts of the project together.
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
#include "segmentlcd.h"
#include "em_chip.h"

#include "globals.h"
#include "clock.h"
#include "ui.h"
#include "us_tx.h"
#include "us_uart.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * data definitions
 *****************************************************************************/
bool          wakeUp  = false ;      /**< Used in main loop and ISR  */




/*******************************************************************************
 **************************   MAIN   *******************************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /*Initialize peripherals */
  InitClocks() ;                // Initialize clock system
  InitButtons() ;               // Initialize GPIO-Buttons
  SegmentLCD_Init(false);       // Initialise LCD
  
  
  /*Write ready at boot up*/
  SegmentLCD_Write("Ready");
  
  
  /* Main loop */
  while(1)
  {
    if (!wakeUp)     // go into enegy mode 1 when nothing is to do
      EMU_EnterEM1();   // wake up with next interrupt 
    wakeUp = false ;
    
    if (GUIstatechanged)
    {
      STATE_INITIALISER();
      GUIstatechanged = false;
    }
    
    if (PB1waspressed)
    {
      ButtonPB1pressed();
      PB1waspressed = false;
    }
    /*
   *   Enter code here to be executed in the main loop
    */
    
  }
 
}// END: main


