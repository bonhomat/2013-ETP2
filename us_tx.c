/**************************************************************************//**
 * @file us_tx.c
 *
 * @brief ultra sonic TX functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-18
 * @version 0.1
 *
 * @verbatim
 * ultra sonic TX used defines, configuration and functions
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
//#include "em_chip.h"

#include "globals.h"

#include "us_tx.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * Static variables
 *****************************************************************************/
uint32_t     counter  = 0;



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


/*******************************************************************************
 * @brief Load TIMER1 registers for continous signal
 ******************************************************************************/
void TX_Start_Signal(void)
{
  TIMER1->CNT = TIMER1_LOAD_VAL;
  TIMER1->CC[0].CTRL = DR_CC_RUN;   // RUN Output on TX Module CH1
  TIMER1->CC[1].CTRL = DR_CC_RUN;   // RUN Output on TX Module CH2
}

/*******************************************************************************
 * @brief Load TIMER1 registers to stop continous signal
 ******************************************************************************/
void TX_Stop_Signal(void)
{
  TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
  TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
}


/*******************************************************************************
 * @brief Load TIMER1 registers for a Burst-Run
 ******************************************************************************/
void TX_Start_Burst(void)
{
  counter = 0;
  TIMER1->CNT = TIMER1_LOAD_VAL;
  TIMER1->CC[2].CTRL = CC2_RUN;
  TIMER1->CC[0].CTRL = DR_CC_RUN;
  TIMER1->CC[1].CTRL = DR_CC_RUN;
}


/*******************************************************************************
 * @brief Load TIMER1 registers for a Burst-Run
 ******************************************************************************/
void TX_Stop_Burst(void)
{
  TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
  TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
  TIMER1->CC[2].CTRL = CC2_STOP;
}


/*******************************************************************************
 * @brief Put TIMER1 into running mode
 ******************************************************************************/
void TX_Timer_Run(void)
{
  // CMU_ClockEnable ( cmuClock_TIMER1, true );  /* Enable clock for TIMER1 */
  TIMER_Enable    ( TIMER1, true );           /* Enable TIMER1 */
}


/*******************************************************************************
 * @brief Stop TIMER1
 ******************************************************************************/
void TX_Timer_Stop(void)
{
  TIMER_Enable    ( TIMER1, false );          /* Disable TIMER1 */
  // CMU_ClockEnable ( cmuClock_TIMER1, false ); /* Disable clock for TIMER1 */
}


/******************************************************************************
 * @brief TIMER1_IRQHandler on active burst/rburst counting Interrupts of CC2
 *
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
  /* Store the interrupt flags before clearing */
  uint16_t intFlags = TIMER1->IF;

  /* Clear the interrupt flags. Only clear the flags that were stored in */
  /* intFlags in case other flags have been set since then */
  TIMER_IntClear( TIMER1, intFlags );

  /* CC2 interrupt occurred */
  if( intFlags & TIMER_IF_CC2 )
  {
    counter++;
    switch (MM_Entry)
    {
      case distance:
        TX_Stop_Burst();
        break;

      case speed:
        // measure speed
        break;

      case testing:
        switch (SM_Testing)
        {
          case tst_measure:
          case sburst:
            if ( counter == BURST_PULSE_CNT || counter == (BURST_PULSE_CNT + 1) )
            {
              // TIMER1->CC[0].CTRL = DL_CC_STOP;            // stop CH1
              // TIMER1->CC[1].CTRL = DH_CC_STOP;            // stop CH2
              // TIMER1->CC[2].CTRL = CC2_STOP;
              TX_Stop_Burst();
              routineactive = false;  // show Program State OFF
              RoutineStateChng = true;
            }
            break;

          case rburst:
            if (counter == BURST_PULSE_CNT)
            {
              TX_Stop_Signal();
            }
            else if ( SM_Testing == rburst && counter == PERIOD_PULSE_CNT )
            {
              // after a cycle restart the cycle again
              TX_Start_Burst();
            }
            break;

          default:
          ;                                              // do nothing State
        }
        break;

      default:
        ;
    }

  }
}// END: TIMER1_IRQHandler



/*******************************************************************************
 * @brief Initialize TIMER1 in Up/Down Count mode with interrupts on overflow
 *
 ******************************************************************************/
void InitTimer1(void)
{

  /* Enable clock for GPIO module */
  CMU_ClockEnable( cmuClock_GPIO, true );

  /* Enable clock for TIMER1 */
  CMU_ClockEnable( cmuClock_TIMER1, true );


  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet( DL_PORT, DL_PIN, gpioModePushPull, DL_OFF_LVL  );
  /* Set CC1 location 3 pin (PD2) as output */
  GPIO_PinModeSet( DH_PORT, DH_PIN, gpioModePushPull, DH_OFF_LVL);

  #define TOP         (F_HFXO/F_TX)/2   // timer 0 top value use F_HFXO, F_TX

  /* Configuring the Capture-Compare-Channels */

  /* Configure CC channel 0 */
  #define CC_CH_0     0
  TIMER1->CC[CC_CH_0].CTRL = DL_CC_STOP;
  #define CC_VAL_CH0  TOP/4
  TIMER_CompareSet( TIMER1, CC_CH_0, CC_VAL_CH0 );

  /* Configure CC channel 1 */
  #define CC_CH_1     1
  TIMER1->CC[CC_CH_1].CTRL = DH_CC_STOP;
  #define CC_VAL_CH1  TOP/4*3
  TIMER_CompareSet( TIMER1, CC_CH_1, CC_VAL_CH1 );

  /* Configure CC channel 2 */
  #define CC_CH_2     2
  TIMER1->CC[CC_CH_2].CTRL = CC2_STOP;
  #define CC_VAL_CH2  TOP/2
  TIMER_CompareSet( TIMER1, CC_CH_2, CC_VAL_CH2 );


  /* Enable overflow interrupt for TIMER1*/
  TIMER_IntEnable( TIMER1, TIMER_IEN_CC2 );


  /* Clear pending TIMER1 interrupts */
  NVIC_ClearPendingIRQ( TIMER1_IRQn );
  /* Enable TIMER1 interrupt vector in NVIC */
  NVIC_EnableIRQ( TIMER1_IRQn );


  /* Route CC0 and CC1 to location 4 (PD6 PD7) and enable pins */
  TIMER1->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC4);


  /* Set Top Value */
  TIMER_TopSet( TIMER1, TOP );

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  timerInit.enable      = false;                   // if timer is active after init
  // timerInit.enable      = true;                   // if timer is active after init
  timerInit.debugRun    = true;                   // if timer runs when CPU in debugmode
  timerInit.prescale    = timerPrescale1;         // prescaler of Timer
  timerInit.clkSel      = timerClkSelHFPerClk;    // CLK source select
  timerInit.fallAction  = timerInputActionNone;   // counter action if falling edge on input
  timerInit.riseAction  = timerInputActionNone;   // counter action if rising edge on input
  timerInit.mode        = timerModeUpDown;        // Mode of timer
  timerInit.dmaClrAct   = false;                  // DMA mode clear or active
  timerInit.quadModeX4  = false;                  // Quadrature decode mode
  timerInit.oneShot     = false;                  // determine if only one count cycle
  timerInit.sync        = false;                  // Start/stop/reload by other timers


  /* Initialize and Configure timer */
  TIMER_Init( TIMER1, &timerInit );

  /* Start timer */

} // END: InitTimer

