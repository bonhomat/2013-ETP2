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

#include "globals.h"
#include "us_tx.h"
#include "us_uart.h"
#include "us_rx.h"
#include "temperature.h"


/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * data definitions
 *****************************************************************************/
state_main    MM_Entry    = init;     /**< GUI-State for main menue     */
state_testing SM_Testing  = top;      /**< GUI-State sub menue testing  */

bool  routineactive     = false;      /**< ON/OFF of functions in States toggled by PB1*/
bool  GUI_StateChange   = false;      /**< */
bool  RoutineStateChng  = false;      /**< */
bool  PB0waspressed     = false;      /**< */
bool  PB1waspressed     = false;      /**< */




/**************************************************************************//**
 * @brief Enable the Button Interrupts
 *
 *****************************************************************************/
void ButtonsEnable(void)
{
  /* Enable GPIO_EVEN interrupt vector in NVIC*/
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Enable GPIO_ODD interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}


/**************************************************************************//**
 * @brief Disable the Button Interrupts
 *
 *****************************************************************************/
void ButtonsDisable(void)
{
  /* Disable GPIO_EVEN interrupt vector in NVIC*/
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);

  /* Disable GPIO_ODD interrupt vector in NVIC */
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
}



/**************************************************************************//**
 * @brief Initialize input and output ports for Buttons
 *
 *****************************************************************************/
void InitButtons(void)
{
  /* Configure Push button 0 as input*/
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);

  /* Configure Push button 1 as input*/
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 1);

  ButtonsEnable();  /* activate Buttons Interrupts */

  /* Set falling edge interrupt for Push Button 0*/
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);

  /* Set falling edge interrupt for Push Button 1 */
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);

}// END: InitButtons





/******************************************************************************
 * @brief Actions for SM_Testing on PB1 press
 *
 *****************************************************************************/
void SM_Testing_PB1pressed(void)
{
  if(SM_Testing != top && SM_Testing != exit_test)
  {
    routineactive = !routineactive;             // Toggle Run/ Stop variable
    RoutineStateChng = true;
  }
  else
  {
    GUI_StateChange = true;
  }

  switch (SM_Testing)
  {
    case top:
      SM_Testing = continious;
      SegmentLCD_EnergyMode(2,1);
      break;

    case continious:                            // Interrupt in continious State
      if (routineactive == true)
      {
        TIMER1->CNT = TIMER1_LOAD_VAL;
        TIMER1->CC[0].CTRL = DR_CC_RUN;         // RUN Output on TX Module CH1
        TIMER1->CC[1].CTRL = DR_CC_RUN;         // RUN Output on TX Module CH2
      }
      else
      {
        TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
        TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
      }
      SegmentLCD_EnergyMode(2,1);
      break;

    case sburst:                                //Interrupt in Burst State
      routineactive = true;
      counter = 0;
      TIMER1->CNT = TIMER1_LOAD_VAL;
      TIMER1->CC[2].CTRL = CC2_RUN;
      TIMER1->CC[0].CTRL = DR_CC_RUN;
      TIMER1->CC[1].CTRL = DR_CC_RUN;
      break;

    case rburst:                                //Interrupt in Repetitive Burst
      if (routineactive == true)
      {
        counter = 0;
        TIMER1->CNT = TIMER1_LOAD_VAL;
        TIMER1->CC[2].CTRL = CC2_RUN;
        TIMER1->CC[0].CTRL = DR_CC_RUN;
        TIMER1->CC[1].CTRL = DR_CC_RUN;
      }
      else
      {
        TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
        TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
        TIMER1->CC[2].CTRL = CC2_STOP;
      }
      break;

    case measure:                               //measure
      {
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);   // show Program State ON
      //DMA_Clr_Buf();
      for(int i = 0; i < DMA_BUFS; i++)
      {
        for(int j=0; j < DMA_BUF_SIZE; j++)
        {
          DMA_buf[i][j] = 0;
        }
      }

      counter = 0;
      TIMER1->CNT = TIMER1_LOAD_VAL;
      TIMER1->CC[2].CTRL = CC2_RUN;
      TIMER1->CC[0].CTRL = DR_CC_RUN;
      TIMER1->CC[1].CTRL = DR_CC_RUN;
      InitADC();
      Measure();
      TIMER1->CC[0].CTRL = DL_CC_STOP;          // stop CH1
      TIMER1->CC[1].CTRL = DH_CC_STOP;          // stop CH2
      TIMER1->CC[2].CTRL = CC2_STOP;
      }
      break;

    case uart:                                  //send data
      routineactive = true;
      SendUart();
      break;

    case exit_test:
      SM_Testing = top;
      routineactive = false;
      SegmentLCD_EnergyMode(2,0);
      break;

    default:
      SegmentLCD_Write("ERROR");                //Interrupt in all other cases
      break;
  }
}






/**************************************************************************//**
 * @brief Button PB1 pressed Routine
 *
 *****************************************************************************/
void ButtonPB1pressed(void)
{
  switch (MM_Entry)
  {
    case distance:
      // measure distance
      break;

    case speed:
      // measure speed
      break;

    case temp:
      // measure temperature
      TempData = getTemperature();
      SegmentLCD_Number(TempData.degrees*100+TempData.fraction);
      break;

    case setting:
      SegmentLCD_EnergyMode(1,1);
      MM_Entry = offset;
      GUI_StateChange = true;
      break;

    case offset:
      // change the offset
      TempData = getTemperature();
      break;

    case n_of_meas:
      // change number of measurements
      break;

    case testing:
      SM_Testing_PB1pressed();
      break;

    case exit_set:
      MM_Entry = distance;
      GUI_StateChange = true;
      SegmentLCD_EnergyMode(1,0);
      break;
  }

}// END: ButtonPB1pressed





/******************************************************************************
 * @brief Initial settings for all states
 *
 *****************************************************************************/
void STATE_INITIALISER(void)
{
  switch (MM_Entry)
  {
    case init:
      SegmentLCD_Write("Ready");
      break;

    case distance:
      SegmentLCD_Write("DIST  m");
      break;

    case speed:
      SegmentLCD_Write("SPEED");
      break;

    case temp:
      InitTSensor( T_SENS_ACTIVE );      // Put temp sensor into shutdown mode
      SegmentLCD_Write("AIRTemp");
      while (TempData.valid == 0)
      {
        TempData = getTemperature();
      }
      SegmentLCD_Number(TempData.degrees*100+TempData.fraction);
      SegmentLCD_Symbol(LCD_SYMBOL_DEGC,1);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
      break;

    case setting:
      InitTSensor( T_SENS_SHUTDOWN );      // Put temp sensor into shutdown mode
      SegmentLCD_NumberOff();
      SegmentLCD_Symbol(LCD_SYMBOL_DEGC,0);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,0);
      SegmentLCD_Write("Set   +");
      break;

    case offset:
      SegmentLCD_Write("OFFSET");
      break;

    case n_of_meas:
      SegmentLCD_Write("NofMeas");
      break;

    case testing:
      switch (SM_Testing)
      {
        case top:
          SegmentLCD_Write("Tests +");
          break;

        case continious:
          InitTimer1() ;                          // Initialize timer 1
          SegmentLCD_Write("CW >>>");
          TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
          TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
          TIMER1->CC[2].CTRL = CC2_STOP;
          routineactive = false;                  // Set routine as not active
          break;

        case sburst:
          InitTimer1() ;                          // Initialize timer 1
          SegmentLCD_Write("Burst");
          TIMER1->CC[0].CTRL = DL_CC_STOP;
          TIMER1->CC[1].CTRL = DH_CC_STOP;
          TIMER1->CC[2].CTRL = CC2_STOP;
          routineactive = false;                  // Set routine as not active
          break;

        case rburst:
          InitTimer1() ;                          // Initialize timer 1
          SegmentLCD_Write("R-Burst");
          routineactive = false;                  // Set routine as not active
          break;

        case measure:
          SegmentLCD_Write("Mess");
          //InitADC();
          routineactive = false;                  // Set routine as not active
          break;

        case uart:
          SegmentLCD_Write("Uart");
          InitUart();                             // Initialise Uart
          routineactive = false;                  // Set routine as not active
          break;

        case exit_test:
          SegmentLCD_Write("Tests X");
          routineactive = false;                  // Set routine as not active
          break;

        default:
          SegmentLCD_Write("missing");
          routineactive = false;                  // Set routine as not active
      }
      break;

    case exit_set:
      SegmentLCD_Write("Set   X");
      break;

    default:
      SegmentLCD_Write("missing");
  }

  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
}// END: STATE_INITIALISER






/******************************************************************************
 * @brief Changes to the next state in the main level GUI
 *
 *****************************************************************************/
 void ButtonPB0pressed(void)
{
  switch (MM_Entry)
  {
    case init:
      MM_Entry = distance;
      GUI_StateChange = true;
      break;

    case distance:
      MM_Entry = speed;
      GUI_StateChange = true;
      break;

    case speed:
      MM_Entry = temp;
      GUI_StateChange = true;
      break;

    case temp:
      MM_Entry = setting;
      GUI_StateChange = true;
      break;

    case setting:
      MM_Entry = distance;
      GUI_StateChange = true;
      break;

    case offset:
      MM_Entry = n_of_meas;
      GUI_StateChange = true;
      break;

    case n_of_meas:
      MM_Entry = testing;
      GUI_StateChange = true;
      break;

    case testing:
      switch (SM_Testing) // change sub-menue
      {
        case top:
          MM_Entry = exit_set;
          break;

        case continious:
          SM_Testing = sburst;
          break;

        case sburst:
          SM_Testing = rburst;
          break;

        case rburst:
          SM_Testing = measure;
          break;

        case measure:
          SM_Testing = uart;
          break;

        case uart:
          SM_Testing = exit_test;
          break;

        case exit_test:
          SM_Testing = continious;
          break;

        default:
          SM_Testing = top;

      } //End: switch

      GUI_StateChange = true;
      break;

    case exit_set:
      MM_Entry = offset;
      GUI_StateChange = true;
      break;

    default:
      MM_Entry = init;
      GUI_StateChange = true;
  }
}



/******************************************************************************
 * @brief UI_Main handels main statemachine changes
 *
 *****************************************************************************/
void UI_Main(void)
{
  if(GUI_StateChange)
  {
    STATE_INITIALISER();
    GUI_StateChange = false;
  }

  if(RoutineStateChng)
  {
    SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
    RoutineStateChng = false;
  }

  if(PB1waspressed)
  {
    ButtonPB1pressed();
    PB1waspressed = false;
    wakeUp = true;
  }


  if(PB0waspressed)
  {
    ButtonPB0pressed();
    PB0waspressed = false;
    wakeUp = true;
  }
}




/*******************************************************************************
 **************************   INTERRUPT HANDLERS   *****************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief Pushbutton 0 GPIO_EVEN_IRQHandler
 *
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  uint16_t gp_flags0 = GPIO->IF & (1<<PB0_PIN); //If GPIO EVEN Interrupt Mask on PB0

  if(gp_flags0)                                 //Use only PB0 interrupt
  {

    PB0waspressed = true;
    GPIO_IntClear(gp_flags0);                    // Clear the interrupt flag PB0

    /* Do not go into EM1 mode after return from ISR */
    wakeUp = true;
  }
} // END: GPIO_EVEN_IRQHandler



/******************************************************************************
 * @brief Pushbutton 1 GPIO_ODD_IRQHandler
 *
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{

  uint16_t gp_flags1 = GPIO->IF & (1<<PB1_PIN);   //If GPIO ODD Interrupt Mask on PB1


  if(gp_flags1)                                   //Use only PB1 interrupt
  {
    PB1waspressed = true;
    GPIO_IntClear(gp_flags1);                     // Clear the interrupt flag PB1

    /* Do not go into EM1 mode after return from ISR */
    wakeUp = true;
  }
}// End: GPIO_ODD_IRQHandler




