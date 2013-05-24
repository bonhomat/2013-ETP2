/**************************************************************************//**
 * @file temperature.c
 *
 * @brief temperature functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-18
 * @version 0.1
 *
 * @verbatim
 * temperature used defines, configuration and functions
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

#include "temperature.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS *********************************
 ******************************************************************************/



/***************************************************************************//**
 * Static variables
 *******************************************************************************/
bool isShutDownTempSensor   = true;   // stores TempSensor state
bool isFirstReadAfterActiv  = false;  // if first read after activate
uint32_t    c_air;        // used for speed calculations updated with temperature reading
TempData_t  TempData;



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief Initialize clock system for temperature reading
 *
 *****************************************************************************/
void InitClkTemperature()
{
  /* Activate high frequency crystal oscillator HFXO */
  #define ON      true    // Enable oscillator
  #define DO_WAIT true    // return when osc is ready to use
  CMU_OscillatorEnable( cmuOsc_HFXO, ON, DO_WAIT );
  CMU_ClockSelectSet  ( cmuClock_HF, cmuSelect_HFXO );

}// End:InitClkTemperature


/**************************************************************************//**
 * @brief Initialize USART1 for SPI communication
 *****************************************************************************/
void InitUsart1SPI(void)
{
  // enable clocks
  CMU_ClockEnable( cmuClock_HFPER,  true );
  CMU_ClockEnable( cmuClock_USART1, true );
  CMU_ClockEnable( cmuClock_GPIO,   true );

  #define CS_TSENSOR_IDLE   1
  #define SCLK_IDLE         0

  /* Configure I/O ports */

  // configure GPIO pins for SPI
  GPIO_PinModeSet( SPI_PORT, SPI_MISO, gpioModeInput, 0 );                      // MISO (MOSI not used!)
  GPIO_PinModeSet( SPI_PORT, SPI_SCLK, gpioModePushPullDrive, SCLK_IDLE );      // CLK
  GPIO_PinModeSet( SPI_PORT, CS_T_SENSOR, gpioModePushPull, CS_TSENSOR_IDLE );  // output for Temp Sensor CS

  USART_InitSync_TypeDef init  = USART_INITSYNC_DEFAULT;        // USART in sync mode, start with default settings
  init.msbf = true;                                             // Most Significant Bit first
  init.baudrate = 2000000;                                      // Baudrate (may be increased if needed )
  init.databits = usartDatabits16;                              // 16 bit frame
  USART_InitSync( USART1, &init );

 } // End InitUsart1SPI

/**************************************************************************//**
 * @brief setup sensor for air temperature => continuous mode
 *****************************************************************************/
void InitTSensor(TempSensorMode_t mode)
{
  // wake temperature sensor by reading 16 bit Manufacture’s/Device ID
  // and write 16 bit 0x0000 for conversion mode continous
  //Route  MISO and SCLK to the GPIO-pins
  USART1->ROUTE =  USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;
  USART1->CTRL &= ~USART_CTRL_AUTOCS;       // Deactivate auto CS for temp sensor
  GPIO_PinOutClear(SPI_PORT, CS_T_SENSOR ); // Activate CS
  USART_TxDouble(USART1, 0x00);             // "read" 16bit from sensor

  while ( !(USART1->STATUS & USART_STATUS_TXC) ) {}

  // Reconfigure SPI-lines, do not route MISO to GPI0 pin
  USART1->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;
  // Reconfigure MISO to be a GPIO output and set it to 0 or 1
  GPIO_PinModeSet(SPI_PORT, SPI_MISO, gpioModePushPullDrive, mode);
  USART_TxDouble(USART1, 0x00);
  while ( !(USART1->STATUS & USART_STATUS_TXC) ) {}
  GPIO_PinOutSet(SPI_PORT, CS_T_SENSOR ); // De-activate CS

  // Make sure RX buffer is now empty
  USART1->CMD = USART_CMD_CLEARRX;

  if (mode == T_SENS_ACTIVE && isShutDownTempSensor == T_SENS_SHUTDOWN )
  {
    // Delay here for about 500ms
  }

  isShutDownTempSensor = mode;

  if (mode == T_SENS_ACTIVE ) isFirstReadAfterActiv = true; // to correct port settings
}



/**************************************************************************//**
* @brief Initialize SPI USART for communication with temperature sensor
*  Activate auto chipselect, set framesize to 16 bit
 *****************************************************************************/
void InitTempSensorComm()
{
  GPIO_PinModeSet( SPI_PORT, SPI_MISO, gpioModeInput, 0 );                  // MISO
  USART1->CTRL |= USART_CTRL_AUTOCS;  // Activate auto CS for temp sensor
  //Route MISO, CS and SCLK to the GPIO-pins
  USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_CSPEN | USART_ROUTE_CLKPEN |
                    USART_ROUTE_LOCATION_LOC1;
  USART1->FRAME = USART_FRAME_STOPBITS_ONE | USART_FRAME_DATABITS_SIXTEEN;  //Set framesize = 16bit

}//InitTempSensorComm()


/**************************************************************************//**
 * @brief get emperature from external sensor
 * @return  temperature result struct
 * @note see datasheet of LM74CIM-3
 *****************************************************************************/
TempData_t  getTemperature(void)
{
  TempData_t result; // Used for return values

  // Wake up sensor if is in shutdown mode
  if (isShutDownTempSensor) InitTSensor( T_SENS_ACTIVE );

  // Wait until a potential previous SPI comm is completed
  while ( !(USART1->STATUS & USART_STATUS_TXC) ) {}

  // Now set up USART for communcating with temp. sensor
  if (isFirstReadAfterActiv)
  {
    InitTempSensorComm();
    isFirstReadAfterActiv = false;
  }

  // Write a dummy word
  USART_TxDouble(USART1, 0x00);     // to generate clock

  // Read received word from RX register
  int16_t temp = USART_RxDouble(USART1);
  temp = temp >> 2;
  result.valid = temp & 1;
  temp = temp >> 1;
  result.raw = temp;
  result.degrees = temp >> 4;
  result.fraction = ( 100*( temp & 0xF ) ) >> 4;  // Convert to 2 digit fraction

  // calculate sound speed in air
  c_air = ( C_INC_DEG * result.raw ) >> 4;
  c_air += C_0_SPEED;
  if( c_air < C_MIN_SPEED || C_MAX_SPEED < c_air || !result.valid )
  {
    // calulation problem, assume 20°C
    c_air = C_INC_DEG * 20;
    c_air += C_0_SPEED;
  }

  return result;
} // getTemperature()



