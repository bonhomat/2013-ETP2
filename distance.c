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

#include "distance.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS *********************************
 ******************************************************************************/

#define DIST_BUF_SIZE  16       // max number of dist measurements in buffer

/***************************************************************************//**
 * Static variables
 *******************************************************************************/
uint16_t  Dist_buf[ DIST_BUF_SIZE ]; // Distance Buffer
int8_t    offset_val = 0;     // offset of the distanc masurement +/-100mm in 5mm steps
uint8_t   destPos = 0;    // destination position in distance array
uint8_t   noMeasurements = 1;  // max val is DIST_BUF_SIZE;



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief get average from the buffer
 * @return average
 *****************************************************************************/
uint16_t average_Dist_buf( uint8_t valsInBuf )
{
  uint32_t avg = 0;   // used for return value
  for( uint8_t i = 0; i < valsInBuf; i++)
  {
    avg += Dist_buf[i];
  }
  avg /= destPos;
  
  return (uint16_t)avg;
}

/**************************************************************************//**
 * @brief get average distance from noSamples measurements
 * @return distance
 *****************************************************************************/
uint16_t getAvgDistance( uint8_t noSamples )
{
  uint32_t  dist; // used for return value
  uint8_t   i;    // internal counter
  destPos = 0;
  
  for( i = 0; i < DIST_BUF_SIZE; i++)   // clear the buffer
  {
    Dist_buf[destPos] = 0;
  }
  
  if(noSamples > DIST_BUF_SIZE) noSamples = DIST_BUF_SIZE;
  
  ButtonsDisable();  // disable user buttons
  
  RTC_tick = false;
  while( !RTC_tick )    // wait for tick
  {
    EMU_EnterEM1();
  }
  
  
  #define MAX_DIST_READINGS   2*DIST_BUF_SIZE   // max number of trials
  for( i = 0; destPos < noSamples && i < 2*noSamples; i++ )
  {
    RTC_tick = false;
    TX_Start_Burst();
    RX_Measure();
    if ( out_of_range )
    {
      // do nothing
    }
    else
    {
      Dist_buf[destPos] = MaxCount; // store uncalibrated distance into buffer
      destPos++;
    }
    while( !RTC_tick )    // wait for tick
    {
      EMU_EnterEM1();
    }
  }
  
  // caluclations
  dist = c_air / 10;
  dist *= average_Dist_buf(destPos);
  dist /= 32000;
  
  dist += offset_val;
  
  ButtonsEnable();  // enable user buttons
  
  return (uint16_t)dist;
} // getDistance()


