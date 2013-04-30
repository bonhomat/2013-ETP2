
/**************************************************************************//**
 * @brief Enable the Button Interrupts
 * 
 *****************************************************************************/
static void ButtonsEnable(void)
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
static void ButtonsDisable(void)
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
static void InitButtons(void)
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
}// END: ButtonPB1pressed





/******************************************************************************
 * @brief Initial settings for all states
 *
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
 * @brief Changes to the next State in the GUI
 *
 *****************************************************************************/
 void GUIState_Switch(void)
{
  switch (GUIState)
  {
    case waiting: 
      GUIState = continuous;    // Go to next State
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
