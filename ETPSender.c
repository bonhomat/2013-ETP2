/***************************************************************************//**
 * @file
 * @brief ETP1 TX signal generator, using 160kHz interrupt, 
 *         
 *        TX1 and TX2 are assigned to GPIO outputs and set/cleared in the ISR
 * @author Dozent: eand@zhaw.ch Students: burrisim@students.zhaw.ch, bonhomat@students.zhaw.ch
 * @version 1.1
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 ZHAW, http://www.zhaw.ch</b>
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
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * Global defines
 *******************************************************************************/
/* Drive port definitions */
#define DL_PORT     gpioPortD   /**< em_lib name of port used for DL */
#define DH_PORT     gpioPortD   /**< em_lib name of port used for DH */
#define DL_PIN      1           /**< Bit/Pin # for DL in above port */
#define DH_PIN      2           /**< Bit/Pin # for DH in above port */
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

#define CC2_RUN       TIMER_CC_CTRL_MODE_OUTPUTCOMPARE
#define CC2_STOP      TIMER_CC_CTRL_MODE_OFF
                      
                      
                                /**< Button port definitions */
#define PB0_PORT   gpioPortD                         //Port D
#define PB0_PIN    8                                 //Button 0 on Board tg
#define PB1_PORT   gpioPortB                         //Port B
#define PB1_PIN    11                                //Button 1 on Board tg


                                /**< Timing definitions */
#define F_HFXO          32000000                     // Crystal oscillator frequency
#define F_TX            40000                        // 40kHz --> TX frequency


                                /**< Counter definitions */
#define BURST_PULSE_CNT   80                          // Pulsewith of burst
#define PERIOD_PULSE_CNT  80000                       // total count in one burst cycle
#define TIMER0_LOAD_VAL   201                         // the value to load in Counter
                                                      //   of Timer0 for next bust

/*******************************************************************************
 * Enums
 *******************************************************************************/

/*******************************************************************************
* @brief Definition of Menu state phases
********************************************************************************/
 typedef enum states
{
  waiting,                      /**< Waiting */
  continuous,                   /**< State ON */
  sburst,                       /**< State single Burst */
  rburst,                       /**< State repetitive burst*/
  init                          /**< Initial state*/
  } menu_state ;


/*****************************************************************************
 * Static data definitions
 *****************************************************************************/
static menu_state  GUIState  = waiting;    /**< Define current GUIState     */
static bool          wakeUp  = false ;     /**< Used in main loop and ISR   */
static uint32_t     counter  = 0;
static bool   routineactive  = false;      /**< ON/OFF of functions in States toggled by PB1*/
static bool GUIstatechanged  = false;      /**< */
static bool   PB1waspresses  = false;      /**< */


/*******************************************************************************
 **************************   INTERRUPT HANDLERS   *****************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief GPIO Interrupt Handler. Writes status to LCD display
 *****************************************************************************/

void GPIO_EVEN_IRQHandler(void)
{ 
  uint16_t gp_flags0 = GPIO->IF & (1<<PB0_PIN); //If GPIO ODD Interrupt Mask on PB1 
  
  if(gp_flags0)                                 //Use only PB1 interrupt
  {
    switch (GUIState)
    {
      case waiting: 
        GUIState = continuous;                  // Go to next State on Interrupt from PB0
        break;
      
      case continuous: 
        GUIState = sburst;
        break;
      
      case sburst:
        GUIState = rburst;
        break;
      
      case rburst: 
        GUIState = waiting;
        break;
      
      default:
        GUIState = waiting;
    } //End: switch
    
    GPIO_IntClear(gp_flags0);                    // Clear the interrupt flag PB0
    GUIstatechanged = true;
    
    /* Do not go into EM1 mode after return from ISR */
    wakeUp = true;
  }
} // End: GPIO_EVEN_IRQHandler

/******************************************************************************
 * @brief Initial Sub functions in GUIState when PB1 is Pressed eg. ON /OFF 
 *****************************************************************************/

void GPIO_ODD_IRQHandler(void)
{

  uint16_t gp_flags1 = GPIO->IF & (1<<PB1_PIN);   //If GPIO ODD Interrupt Mask on PB1 

  
  if(gp_flags1)                                   //Use only PB1 interrupt
  {
    PB1waspresses = true;
    GPIO_IntClear(gp_flags1);                     // Clear the interrupt flag PB1
  }
}// End: GPIO_ODD_IRQHandler

void ButtonPB1pressesd(void)
{
  routineactive = !routineactive;                 // Toggle Run/ Stop variable
  
  switch (GUIState)
  {
    
    case waiting:                                 // Interrupt in Waiting State
      break;
        
    case continuous:                              // Interrupt in Continuous State
      if (routineactive == true)
      {
        TIMER0->CNT = TIMER0_LOAD_VAL;
        TIMER0->CC[0].CTRL = DR_CC_RUN;           // RUN Output on TX Module CH1
        TIMER0->CC[1].CTRL = DR_CC_RUN;           // RUN Output on TX Module CH2
      }
      else
      {
        TIMER0->CC[0].CTRL = DL_CC_STOP;          // Stop Output on TX Module CH1
        TIMER0->CC[1].CTRL = DH_CC_STOP;          // Stop Output on TX Module CH2
      }
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
      break;

    case sburst:                                  //Interrupt in Burst State
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);     // show Program State ON
      counter = 0;
      TIMER0->CNT = TIMER0_LOAD_VAL;
      TIMER0->CC[2].CTRL = CC2_RUN;
      TIMER0->CC[0].CTRL = DR_CC_RUN;
      TIMER0->CC[1].CTRL = DR_CC_RUN;
      break;
    
    case rburst:                                  //Interrupt in Repetitive Burst
      if (routineactive == true)
      {             
        counter = 0;
        TIMER0->CNT = TIMER0_LOAD_VAL;
        TIMER0->CC[2].CTRL = CC2_RUN;
        TIMER0->CC[0].CTRL = DR_CC_RUN;
        TIMER0->CC[1].CTRL = DR_CC_RUN;
      }
      else
      {
        TIMER0->CC[0].CTRL = DL_CC_STOP;          // Stop Output on TX Module CH1
        TIMER0->CC[1].CTRL = DH_CC_STOP;          // Stop Output on TX Module CH2
        TIMER0->CC[2].CTRL = CC2_STOP;
      }
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
      break;
      
    default:
      SegmentLCD_Write("ERROR");                   //Interrupt in all other cases
      break;
  }  
}// END: ButtonPB1pressesd




/******************************************************************************
 * @brief Initial settings for all States
 *****************************************************************************/
 
void STATE_INITIALISER(void)
{
  switch (GUIState)
  {
    case waiting: 
      SegmentLCD_Write("Idle");
      TIMER0->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
      TIMER0->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
      TIMER0->CC[2].CTRL = CC2_STOP;
      routineactive = false;                  // Set routine as not active
      break;
      
    case continuous: 
      SegmentLCD_Write("CW >>>");
      routineactive = false;                  // Set routine as not active
      break;
      
    case sburst:
      SegmentLCD_Write("Burst");
      TIMER0->CC[0].CTRL = DL_CC_STOP;
      TIMER0->CC[1].CTRL = DH_CC_STOP;
      TIMER0->CC[2].CTRL = CC2_STOP;
      routineactive = false;                  // Set routine as not active
      break;
      
    case rburst: 
      SegmentLCD_Write("R-Burst");
      routineactive = false;                  // Set routine as not active
      break;
      
    default:
      SegmentLCD_Write("ERROR");
      routineactive = false;                  // Set routine as not active
      
  }
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
}// END: STATE_INITIALISER






/******************************************************************************
 * @brief Initial Sub functions in GUIState on active burst/rburst counting Interrupts of CC2
 *****************************************************************************/

void TIMER0_IRQHandler(void)
{
  /* Store the interrupt flags before clearing */
  uint16_t intFlags = TIMER0->IF;
  
  /* Clear the interrupt flags. Only clear the flags that were stored in */
  /* intFlags in case other flags have been set since then */
  TIMER_IntClear(TIMER0, intFlags);
  
  /* CC2 interrupt occurred */
  if(intFlags & TIMER_IF_CC2)
  {
    counter++;
    switch(GUIState)
    {
      case sburst:
        if (counter == BURST_PULSE_CNT)
        {
          TIMER0->CC[0].CTRL = DL_CC_STOP;            // stop CH1
          TIMER0->CC[1].CTRL = DH_CC_STOP;            // stop CH2
          TIMER0->CC[2].CTRL = CC2_STOP;
          SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 0);     // show Program State OFF
        }
        break;
      
      case rburst:
        if (counter == BURST_PULSE_CNT)
        {
          TIMER0->CC[0].CTRL = DL_CC_STOP;          // stop CH1
          TIMER0->CC[1].CTRL = DH_CC_STOP;          // stop CH2
        }
        else if (GUIState == rburst && counter == PERIOD_PULSE_CNT)
        {
          // after a cycle restart the cycle again
          counter = 0;
          TIMER0->CNT = TIMER0_LOAD_VAL;
          TIMER0->CC[0].CTRL = DR_CC_RUN;
          TIMER0->CC[1].CTRL = DR_CC_RUN;
        }
        break;
      
      default:
      ;                                              // do nothing State
    }
    
  }
}// END: TIMER0_IRQHandler




/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/




/*******************************************************************************
 * @brief Initialize TIMER0 in Up/Down Count mode and to give interrupts on
 * overflow
 ******************************************************************************/
static void InitTimer0()
{  
 
  
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  
  
  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet(DL_PORT, DL_PIN, gpioModePushPull, DL_OFF_LVL);
  /* Set CC1 location 3 pin (PD2) as output */
  GPIO_PinModeSet(DH_PORT, DH_PIN, gpioModePushPull, DH_OFF_LVL);
  
  #define TOP (F_HFXO/F_TX)/2         // timer 0 top value use F_HFXO, F_TX
  
  /* Configuring the Capture-Compare-Channels */
  
  /* Configure CC channel 0 */
  #define CC_CH_0     0
  TIMER0->CC[CC_CH_0].CTRL = DL_CC_STOP;
  #define CC_VAL_CH0  TOP/4
  TIMER_CompareSet(TIMER0, CC_CH_0, CC_VAL_CH0);
  
  /* Configure CC channel 1 */
  #define CC_CH_1     1
  TIMER0->CC[CC_CH_1].CTRL = DH_CC_STOP;
  #define CC_VAL_CH1  TOP/4*3
  TIMER_CompareSet(TIMER0, CC_CH_1, CC_VAL_CH1);
  
  /* Configure CC channel 2 */
  #define CC_CH_2     2
  TIMER0->CC[CC_CH_2].CTRL = CC2_STOP;
  #define CC_VAL_CH2  TOP/2
  TIMER_CompareSet(TIMER0, CC_CH_2, CC_VAL_CH2);
  
  
  /* Enable overflow interrupt for TIMER0*/
  TIMER_IntEnable(TIMER0, TIMER_IEN_CC2);
  
  
  /* Clear pending TIMER0 interrupts */
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  
  /* Route CC0 and CC1 to location 3 (PD1) and enable pins */
  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC3);
  
  
  /* Set Top Value */
  TIMER_TopSet(TIMER0, TOP); 
  
  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  
  timerInit.enable      = true;                   // if timer is active after init
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
  TIMER_Init(TIMER0, &timerInit);
  
  
  /* Start timer */
 
} // End: InitTimer


/**************************************************************************//**
 * @brief Initialize input and output ports 
 * 
 *****************************************************************************/
static void InitButtons()
{ 
  /* Configure Push button 0 as input*/
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
  
  /* Configure Push button 1 as input*/
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 1);
  
  /* Enable GPIO_EVEN interrupt vector in NVIC*/
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
   /* Enable GPIO_ODD interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  /* Set falling edge interrupt for Push Button 0*/
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
  
  /* Set falling edge interrupt for Push Button 1 */
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);
 
}// END: InitButtons



/**************************************************************************//**
 * @brief Initialize GPIO clock
 * 
 *****************************************************************************/
void InitGPIOClk()
{
 
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

}//END: InitGPIOClk

/**************************************************************************//**
 * @brief Initialize clock system
 * 
 *****************************************************************************/
void InitClk()
{
  /* Activate high frequency crystal oscillator HFXO */
  #define ON      true                      	// Enable oscillator
  #define DO_WAIT true                      	// return when osc is ready to use
  CMU_OscillatorEnable( cmuOsc_HFXO, ON, DO_WAIT);
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO);

}// End: InitClk





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
  InitClk();                         // Initialize clock system
  InitGPIOClk();                     // Initialize clock GPIO
  InitButtons();                     // Initialize GPIO-Buttons
  InitTimer0();                      // Initialize timer 0
  SegmentLCD_Init(false);            // Initialize LCD 
  
  
  /*Write ready at boot up*/
  SegmentLCD_Write("Ready");
  
  
  /* Main loop */
  while(1)
  {
    if (!wakeUp)                     // go into energy mode 1 when nothing is to do
      EMU_EnterEM1();                // wake up with next interrupt 
    wakeUp = false;
    
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
    
  }
 
}// END: main
