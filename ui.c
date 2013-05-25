/**************************************************************************//**
 * @file ui.c
 *
 * @brief User Interface functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-18
 * @version 0.1
 *
 * @verbatim
 * User Interface used defines, configuration and functions
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

#include "globals.h"
#include "us_tx.h"
#include "us_uart.h"
#include "us_rx.h"
#include "temperature.h"
#include "distance.h"
#include "speed.h"

/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * Static variables
 *****************************************************************************/
state_main    MM_Entry    = init;     /**< GUI-State for main menue     */
state_testing SM_Testing  = top;      /**< GUI-State sub menue testing  */

bool  routineactive     = false;      /**< ON/OFF of functions in States toggled by PB1*/
bool  GUI_StateChange   = false;      /**< */
bool  RoutineStateChng  = false;      /**< */
bool  PB0waspressed     = false;      /**< */
bool  PB1waspressed     = false;      /**< */



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


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
        TX_Start_Signal();
      }
      else
      {
        TX_Stop_Signal();
      }
      SegmentLCD_EnergyMode(2,1);
      break;

    case sburst:                                //Interrupt in Burst State
      routineactive = true;
      TX_Start_Burst();
      break;

    case rburst:                                //Interrupt in Repetitive Burst
      if (routineactive == true)
      {
        TX_Start_Burst();
      }
      else
      {
        TX_Stop_Burst();
      }
      break;

    case tst_measure:                               //test measure
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);

      for(int i = 0; i < DMA_BUFS; i++)
      {
        for(int j=0; j < DMA_BUF_SIZE; j++)
        {
          DMA_buf[i][j] = 0;
        }
      }

      TX_Start_Burst();
      // InitADC();
      RX_Measure();
      if ( out_of_range )
      {
        SegmentLCD_Number(8888); 
        SegmentLCD_Symbol(LCD_SYMBOL_DP10,0);
      }
      else
      {
        // output of uncalibrated distance
        SegmentLCD_Number(MaxCount/10);           /* Write value out on display */
        SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
      }
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 0);
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
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
      SegmentLCD_Number( getAvgDistance(noMeasurements)/10 );
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 0);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
      // RoutineStateChng = false;
      // routineactive = true;  
      break;

    case speed:
      // measure speed
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
      SegmentLCD_Number( getAvgSpeed() );
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 0);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
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
      offset_val = offset_val + 5;
      if (offset_val > 100) offset_val = -100;
      SegmentLCD_Number(offset_val);
      break;

    case n_of_meas:
      // change number of measurements
      noMeasurements = noMeasurements + 1;
      if (noMeasurements > 16) noMeasurements = 1;
      SegmentLCD_Number(noMeasurements);
      break;

    case testing:
      SM_Testing_PB1pressed();
      break;

    case exit_set:
      MM_Entry = distance;
      GUI_StateChange = true;
      SegmentLCD_EnergyMode(1,0);
      break;
      
    default:
      ;
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
      TX_Timer_Run();
      TX_Stop_Burst();
      SegmentLCD_NumberOff();
      SegmentLCD_Symbol(LCD_SYMBOL_DEGC,0);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,0);
      SegmentLCD_Write("DIST  m");
      InitADC();
      routineactive = false;
      break;

    case speed:
      TX_Timer_Run();
      TX_Stop_Burst();
      SegmentLCD_Write("SPEED");
      InitADC();
      routineactive = false;
      break;

    case temp:
      TX_Stop_Burst();
      InitTSensor( T_SENS_ACTIVE );      // Put temp sensor into active mode
      SegmentLCD_Write("AIRTemp");
      RTC_cntr = 0;
      while ( RTC_cntr <= 10 && !PB0waspressed )   // wait 500ms
      {
        EMU_EnterEM1();
      }
      if (PB0waspressed)  break;
      TempData = getTemperature();
      SegmentLCD_Number(TempData.degrees*100+TempData.fraction);
      SegmentLCD_Symbol(LCD_SYMBOL_DEGC,1);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,1);
      routineactive = false;
      break;

    case setting:
      InitTSensor( T_SENS_SHUTDOWN );      // Put temp sensor into shutdown mode
      SegmentLCD_NumberOff();
      SegmentLCD_Symbol(LCD_SYMBOL_DEGC,0);
      SegmentLCD_Symbol(LCD_SYMBOL_DP10,0);
      SegmentLCD_Write("Set   +");
      routineactive = false;
      break;

    case offset:
      SegmentLCD_Write("OFFSET");
      SegmentLCD_Number(offset_val);
      routineactive = false;
      break;

    case n_of_meas:
      SegmentLCD_Write("NofMeas");
      SegmentLCD_Number(noMeasurements);
      routineactive = false;
      break;

    case testing:
      switch (SM_Testing)
      {
        case top:
          SegmentLCD_NumberOff();
          SegmentLCD_Write("Tests +");
          routineactive = false;
          break;

        case continious:
          TX_Stop_Burst();
          TX_Timer_Run();                         // activate timer 1
          SegmentLCD_Write("CW >>>");
          routineactive = false;                  // Set routine as not active
          break;

        case sburst:
          TX_Stop_Burst();
          TX_Timer_Run();                         // activate timer 1
          SegmentLCD_Write("Burst");
          routineactive = false;                  // Set routine as not active
          break;

        case rburst:
          TX_Stop_Burst();
          TX_Timer_Run();                         // activate timer 1
          SegmentLCD_Write("R-Burst");
          routineactive = false;                  // Set routine as not active
          break;

        case tst_measure:
          TX_Stop_Burst();
          //NVIC_DisableIRQ(RTC_IRQn);              // disable RTC Interrupts
          TX_Timer_Run();                         // activate timer 1
          SegmentLCD_Write("Mess");
          InitADC();
          routineactive = false;                  // Set routine as not active
          break;

        case uart:
          DMA_Reset();
          TX_Timer_Stop();                        // disable RTC Interrupts
          SegmentLCD_Write("Uart");
          InitUart();                             // Initialise Uart
          routineactive = false;                  // Set routine as not active
          break;

        case exit_test:
          //NVIC_EnableIRQ(RTC_IRQn);               // enable RTC Interrupts
          TX_Timer_Stop();
          SegmentLCD_NumberOff();                 // clear value on display
          SegmentLCD_Symbol(LCD_SYMBOL_DP10,0);
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
          SM_Testing = tst_measure;
          break;

        case tst_measure:
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




