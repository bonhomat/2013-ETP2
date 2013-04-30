

/******************************************************************************
 * @brief TIMER0_IRQHandler on active burst/rburst counting Interrupts of CC2
 *
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
 * @brief Initialize TIMER0 in Up/Down Count mode with interrupts on overflow
 *
 ******************************************************************************/
static void InitTimer0(void)
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
 
} // END: InitTimer

