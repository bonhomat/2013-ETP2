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
#include "us_uart.h"
#include "us_rx.h"

/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/


/*****************************************************************************
 * data definitions
 *****************************************************************************/
menue_state GUIState  = waiting;     /**< Define current GUIState  */
bool   routineactive  = false;       /**< ON/OFF of functions in States toggled by PB1*/  
bool GUIstatechanged  = false;       /**< */
bool   PB1waspressed  = false;       /**< */




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




/**************************************************************************//**
 * @brief Button PB1 pressed Routine
 * 
 *****************************************************************************/
void ButtonPB1pressed(void)
{
  routineactive = !routineactive;                 // Toggle Run/ Stop variable
  
  switch (GUIState)
  {
    case continious:                              // Interrupt in continious State
      if (routineactive == true)
      {
        TIMER1->CNT = TIMER1_LOAD_VAL;
        TIMER1->CC[0].CTRL = DR_CC_RUN;           // RUN Output on TX Module CH1
        TIMER1->CC[1].CTRL = DR_CC_RUN;           // RUN Output on TX Module CH2
      }
      else
      {
        TIMER1->CC[0].CTRL = DL_CC_STOP;          // Stop Output on TX Module CH1
        TIMER1->CC[1].CTRL = DH_CC_STOP;          // Stop Output on TX Module CH2
      }
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
      break;

    case sburst:                                  //Interrupt in Burst State
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);     // show Program State ON
      counter = 0;
      TIMER1->CNT = TIMER1_LOAD_VAL;
      TIMER1->CC[2].CTRL = CC2_RUN;
      TIMER1->CC[0].CTRL = DR_CC_RUN;
      TIMER1->CC[1].CTRL = DR_CC_RUN;
      break;
    
    case rburst:                                  //Interrupt in Repetitive Burst
      if (routineactive == true)
      {             
        //counter = 0;
        TIMER1->CNT = TIMER1_LOAD_VAL;
        TIMER1->CC[2].CTRL = CC2_RUN;
        TIMER1->CC[0].CTRL = DR_CC_RUN;
        TIMER1->CC[1].CTRL = DR_CC_RUN;
      }

      else
      {
        TIMER1->CC[0].CTRL = DL_CC_STOP;          // Stop Output on TX Module CH1
        TIMER1->CC[1].CTRL = DH_CC_STOP;          // Stop Output on TX Module CH2
        TIMER1->CC[2].CTRL = CC2_STOP;
      }
      SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
      break;
    
	case uart:                                  //send data
      {
	  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);   // show Program State ON
      sendUart();
	  }
      break;      

	case measure:                                  //measure
      {
	  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);   // show Program State ON
      Measure();
	  }
      break; 
	  
	default:
      SegmentLCD_Write("ERROR");                   //Interrupt in all other cases
      break;
  }  
}// END: ButtonPB1pressed





/******************************************************************************
 * @brief Initial settings for all states
 *
 *****************************************************************************/
void STATE_INITIALISER(void)
{
  switch (GUIState)
  {   
    case continious:
      InitTimer1() ;                          // Initialize timer 0
      SegmentLCD_Write("CW >>>");
      TIMER1->CC[0].CTRL = DL_CC_STOP;        // Stop Output on TX Module CH1
      TIMER1->CC[1].CTRL = DH_CC_STOP;        // Stop Output on TX Module CH2
      TIMER1->CC[2].CTRL = CC2_STOP;
      routineactive = false;                  // Set routine as not active
      break;
      
    case sburst:
      InitTimer1() ;                          // Initialize timer 0
      SegmentLCD_Write("Burst");
      TIMER1->CC[0].CTRL = DL_CC_STOP;
      TIMER1->CC[1].CTRL = DH_CC_STOP;
      TIMER1->CC[2].CTRL = CC2_STOP;
      routineactive = false;                  // Set routine as not active
      break;
      
    case rburst: 
      InitTimer1() ;                          // Initialize timer 0
      SegmentLCD_Write("R-Burst");
      routineactive = false;                  // Set routine as not active
      break;

    case uart: 
      SegmentLCD_Write("Uart");
	  initUart();			                  // Initialise Uart
      routineactive = false;                  // Set routine as not active
      break;
	  
    case measure: 
      SegmentLCD_Write("Mess");
	  InitADC();

      routineactive = false;                  // Set routine as not active
      break;	  
      
    default:
      SegmentLCD_Write("ERROR");
      routineactive = false;                  // Set routine as not active
      
  }
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, (int)routineactive);  // show Program State
}// END: STATE_INITIALISER




/******************************************************************************
 * @brief Changes to the next State in the GUI
 *
 *****************************************************************************/
 void GUIState_Switch(void)
{
  switch (GUIState)
  {
    case continious: 
      GUIState = sburst;
      break;
    
    case sburst:
      GUIState = rburst;
      break;
    
    case rburst: 
      GUIState = uart;
      break;
    
	case uart: 
      GUIState = measure;
      break;
	case measure:
	  GUIState = continious;
    default:
      GUIState = continious;
  } //End: switch
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
    GUIState_Switch();
    
    GPIO_IntClear(gp_flags0);                    // Clear the interrupt flag PB0
    GUIstatechanged = true;
    
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
  }
}// End: GPIO_ODD_IRQHandler
