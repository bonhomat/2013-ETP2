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
  InitClk() ;                   // Initialize clock system
  InitGPIOClk() ;               // Initialize clock GPIO
  InitButtons() ;               // Initialize GPIO-Buttons
  InitTimer0() ;                // Initialize timer 0
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
    
    if (PB1waspresses)
    {
      ButtonPB1pressesd();
      PB1waspresses = false;
    }
    /*
   *   Enter code here to be executed in the main loop
    */
    
  }
 
}// END: main


