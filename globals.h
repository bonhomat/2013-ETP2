/**************************************************************************//**
 * @file globals.h
 *
 * @brief global defines, structs and variables
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-20
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
 
                    /**< Drive port definitions */
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

#define CC2_RUN     TIMER_CC_CTRL_MODE_OUTPUTCOMPARE
#define CC2_STOP    TIMER_CC_CTRL_MODE_OFF

                    /**< SPI port definitions */
//#define SPI_MOSI    0         // Tx pin = MOSI -> not connected to SPI device!
#define SPI_MISO    1           // Rx pin = MISO
#define SPI_SCLK    2           // CLK pin = SCLK
#define CS_T_SENSOR 3           // CS pin for Temp Sensor
//Note PortD Pin 3 is used for AutoCS functionality
#define SPI_PORT    gpioPortD   // SPI Port definition

                    /**< Button port definitions */
#define PB0_PORT    gpioPortD   //Port D
#define PB0_PIN     8           //Button 0 on Board tg
#define PB1_PORT    gpioPortB   //Port B
#define PB1_PIN     11          //Button 1 on Board tg


                    /**< Timing definitions */
#define F_HFXO      32000000    // Crystal oszillator frequency
#define F_TX        40000       // 40kHz --> TX frequency


                    /**< Counter definitions */
#define BURST_PULSE_CNT     80    // Pulsewith of burst was 80
#define PERIOD_PULSE_CNT    80000 // total count in one burst cycle
#define TIMER1_LOAD_VAL     201   // the value to load in Counter
                                  //  of Timer1 for next burst

                    /**< DMA definitions */
#define DMA_TRANSFER_COUNT  60 //40 ///< x DMA transfers timeout for overdistance >60ms,10m
#define DMA_BUF_SIZE        256     // x entries in one DMA buffer
#define DMA_BUFS            4       // x DMA buffers


                  /**< sound speed in air definitions */
#define C_0_SPEED   331300    // speed of sound in mm/s in air at 0°C
#define C_INC_DEG   606       // speed of sound increment in mm/s for 1°C rise
#define C_MIN_SPEED 300000    // at about -40°C in mm/s
#define C_MAX_SPEED 400000    // at about 100°C in mm/s
                  
/*******************************************************************************
 * Enums
 *******************************************************************************/

 
/*******************************************************************************
* @brief Definition of states for the testing menue
********************************************************************************/
typedef enum states_top
{
  init,         /**< Init-State after boot or reset       */
  distance,     /**< Top-State for change to other menue  */
  speed,        /**< State on/continious wave             */
  temp,         /**< State single burst                   */
  setting,      /**< State continious burst               */
  offset,       /**< State uart transmit                  */
  n_of_meas,    /**< State number of measures             */
  testing,      /**< State testing                        */
  exit_set,     /**< State exit setting                   */
} state_main;

/*******************************************************************************
* @brief Definition of states for the testing menue
********************************************************************************/
typedef enum states_sub
{
  top,                          /**< Top-State for change to other menue  */
  continious,                   /**< State on/continious wave             */
  sburst,                       /**< State single burst                   */
  rburst,                       /**< State continious burst               */
  tst_measure,                      /**< single measurement                   */  
  uart,                         /**< State uart transmit                  */
  exit_test,                    /**< State exit from tests                */
} state_testing;




/*******************************************************************************
 * Structs
 *******************************************************************************/

 
 /*******************************************************************************
* @brief Definition of the temperature read return object
********************************************************************************/
typedef struct        // Represents temperature data read from sensor
{                     // in different formats
  int16_t raw;        // temp value in 1/16 degrees celsius
  int16_t degrees;    // Integer value in degrees celsius
  uint8_t fraction;   // Two digit fractional value
  bool valid;         // True if conversion terminated successfully 
} TempData_t;
 

 
 
/*****************************************************************************
 * global variable references
 *****************************************************************************/
extern state_testing  SM_Testing;
extern state_main     MM_Entry;
extern bool           wakeUp;
extern uint32_t       TX_counter;
extern uint16_t       RTC_cntr;
extern bool           RTC_tick;
extern bool           PB0waspressed;
extern bool           PB1waspressed;
extern bool           GUI_StateChange;
extern bool           RoutineStateChng;
extern bool           routineactive;
extern TempData_t     TempData;
extern uint32_t       c_air;
extern uint16_t       DMA_buf[DMA_BUFS][DMA_BUF_SIZE];
extern uint16_t       DMA_buf_last;
extern uint16_t       DMA_buf_current;
extern uint16_t       MaxCount;
extern bool           out_of_range;
extern int8_t         offset_val;
extern uint8_t        noMeasurements;
#endif

