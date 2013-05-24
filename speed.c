/**************************************************************************//**
 * @file distance.c
 *
 * @brief distance functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-18
 * @version 0.1
 *
 * @verbatim
 * distance used defines, configuration and functions
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
#include "em_usart.h"
#include "em_system.h"
#include "em_timer.h"
#include "segmentlcd.h"

#include "globals.h"
#include "us_tx.h"
#include "us_rx.h"
#include "ui.h"

#include "speed.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS *********************************
 ******************************************************************************/

#define SPEED_BUF_SIZE  5       // max number of dist measurements in buffer

/***************************************************************************//**
 * Static variables
 *******************************************************************************/
uint16_t  Dist_Speed_buf[ SPEED_BUF_SIZE ]; // Distance Buffer



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief get average from the buffer
 * @return average
 *****************************************************************************/
uint16_t average_Speed_buf( uint8_t valsInBuf )
{
  int32_t avg = 0;   // used for return value
  uint8_t noVals = 0;
  for( uint8_t i = 0; i < valsInBuf-1; i++)
  {
    if (Dist_Speed_buf[i]>0)
    {
      avg += Dist_Speed_buf[i]-Dist_Speed_buf[i+1];
      noVals++;
    }
  }
  avg /= noVals;
  
  return (int16_t)avg;
}

/**************************************************************************//**
 * @brief get average speed from 5 measurements
 * @return distance
 *****************************************************************************/
int16_t getAvgSpeed(void)
{
  int32_t   speed;    // used for return value
  uint8_t   i;        // internal counter
  uint8_t   destPos = 0;
  
  // ButtonsDisable();  // disable user buttons
  
  
  #define SPEED_DIST_READINGS   5   // of dist-readings
  for( i = 0; i < SPEED_DIST_READINGS; i++ )
  {
    RTC_tick = false;
    TX_Start_Burst();
    RX_Measure();
    if ( out_of_range )
    {
      // do nothing
      Dist_Speed_buf[destPos] = 0;
    }
    else
    {
      Dist_Speed_buf[destPos] = MaxCount; // store uncalibrated distance into buffer
    }
    destPos++;
    while( !RTC_tick )    // wait for tick
    {
      EMU_EnterEM1();
    }
  }
  
  // caluclations
  speed = c_air / 10;
  speed *= average_Speed_buf(5);
  speed /= 32000;
  
  // ButtonsEnable();  // enable user buttons
  
  return (int16_t)speed;
} // getDistance()


