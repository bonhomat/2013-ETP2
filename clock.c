



/**************************************************************************//**
 * @brief Initialize GPIO clock
 * 
 *****************************************************************************/
void InitGPIOClk(void)
{
 
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

}//END: InitGPIOClk

/**************************************************************************//**
 * @brief Initialize clock system
 * 
 *****************************************************************************/
void InitClk(void)
{
  /* Activate high frequency crystal oscillator HFXO */
  #define ON      true                      	// Enable oscillator
  #define DO_WAIT true                      	// return when osc is ready to use
  CMU_OscillatorEnable( cmuOsc_HFXO, ON, DO_WAIT );
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );

}// End: InitClk


/**************************************************************************//**
 * @brief Initializes all needed clocks
 * 
 *****************************************************************************/
void InitClocks(void)
{
  InitClk();
  InitGPIOClk();
}