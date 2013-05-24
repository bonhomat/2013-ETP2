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
#include "us_rx.h"
#include "temperature.h"



/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * Static variables
 *****************************************************************************/
bool    wakeUp = false;         /**< Used in main loop and ISR  */
uint32_t tcntr = 0; /* test counter for break while waiting for temp */




/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief Setup for energy measurments of device
 *****************************************************************************/
void setupSWO(void)
{
  /* Enable GPIO Clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY) || defined(_EFM32_LEOPARD_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  DWT->CTRL = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  TPI->ACPR = 0xf;
  /* Set protocol to NRZ */
  TPI->SPPR = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}



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

  setupSWO();

  /*Initialize peripherals */
  InitClocks();                   // Initialize clock system
  InitTimer1();                   // Initialize Timer1 for TX
  InitButtons();                  // Initialize GPIO-Buttons
  SegmentLCD_Init(false);         // Initialise LCD


  InitUsart1SPI();                // Initialise SPI to temp sensor
  InitTSensor( T_SENS_SHUTDOWN ); // Put temp sensor into shutdown mode

  #define TEMP_READ_MAX_TRIES 50000 // maximal number of retries to read temperature
  for ( tcntr = 0; !TempData.valid && tcntr < TEMP_READ_MAX_TRIES ; tcntr++)
  {
    TempData = getTemperature();
  }
  InitTSensor( T_SENS_SHUTDOWN ); // Put temp sensor into shutdown mode


  if (!TempData.valid)
  {
    SegmentLCD_Write("ErrTSen");
    SegmentLCD_Number(tcntr);
  }
  else
  {
    /*Write ready at boot up*/
    SegmentLCD_Write("Ready");
    SegmentLCD_EnergyMode(0,1);
    SegmentLCD_Number(TempData.degrees*100+TempData.fraction);
    SegmentLCD_Symbol(LCD_SYMBOL_DEGC,1);
    SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
  }

  /* Main loop */
  while(1)
  {
    if (!wakeUp)        // go into enegy mode 1 when nothing is to do
      EMU_EnterEM1();   // wake up with next interrupt
    wakeUp = false;

    UI_Main();

  }

}// END: main


