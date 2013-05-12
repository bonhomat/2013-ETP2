
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
#include <stdbool.h>
#include "em_device.h"
#include "em_dma.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_int.h"
#include "dmactrl.h"

#include "globals.h"




/*****************************************************************************
 * data definitions
 *****************************************************************************/
/* ADC settings */
#define ADC_samp_freq                 160000    ///< ADC sampling frequency
#define DMA_CHANNEL_ADC               1         ///< DMA channel for ADC
#define ADC_Port                      gpioPortD ///< DMA Port 
#define ADC_Pin                       4         ///< DMA Pin 

/* ADC => DMA buffers and related */
DMA_CB_TypeDef cbn;                             ///< callback structure
volatile bool transferActive;                   ///< transfer flag
#define DMA_TRANSFER_COUNT            5         ///< x DMA transfers
#define DMA_BUFFER_SIZE               20        ///< x entries in one DMA buffer
#define DMA_BUFFER_COUNT              4         ///< x DMA buffers >= 2
uint16_t DMA_buffer[DMA_BUFFER_COUNT][DMA_BUFFER_SIZE];   ///< the data
uint16_t DMA_buffer_last;                       ///< buffer ready for processing
uint16_t DMA_buffer_current;                    ///< buffer is currently filled
uint16_t DMA_buffer_next;                       ///< buffer will be used next

uint16_t averageValue[DMA_TRANSFER_COUNT];      ///< processed data
 

 /**************************************************************************//**
 * @brief Calculate average value of one DMA-buffer
 * @param [in] pointer to data array
 * @return average of array data
 *****************************************************************************/
uint16_t calculateAverage(uint16_t sampleNumber, uint16_t* dmaBuffer) 
{
  uint32_t sum = 0;
  for (uint16_t i = 0; i < sampleNumber; i++) {
    sum += dmaBuffer[i];
  }
  sum = sum / sampleNumber;
  return sum;
}

/**************************************************************************//**
 * @brief  ADC Interrupt handler
 * @par
 *****************************************************************************/
void ADC0_IRQHandler(void) {
  ADC_IntClear(ADC0, ADC_IFC_SINGLEOF);         /* Clear interrupt flag */
  //while(1){
    /** ERROR: ADC Result overflow has occured.
     * This indicates that the DMA is not able to keep up with the ADC sample
     * rate and that a samples has been written to the ADC result registers
     * before the DMA was able to fetch the previous result. */ 
  //}
}

/**************************************************************************//**
 * @brief  Call-back called when DMA transfer is complete
 * @param [in] DMA channel
 * @param [in] DMA descriptor: primary (true) or alternate (false)
 * @param [in] user pointer
 * @par
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user) {
  (void) user;
  
  static int transfernumber = 0;      /* number of transfered buffers */
  
  transfernumber++;                   /* Keeping track of the number of transfered buffers */
  
  //LEDon();                           /* for debugging and timing check*/        //delete
  
  /* numbers of DMA buffers in use */
  DMA_buffer_last = (transfernumber -1) % DMA_BUFFER_COUNT;
  DMA_buffer_current = (transfernumber) % DMA_BUFFER_COUNT;
  DMA_buffer_next = (transfernumber+1) % DMA_BUFFER_COUNT;
  
  //if (transfernumber == 4) { RCtoggle(); } /* ramp up then down for debugging */
    
  /* Let the transfer be repeated a few times to illustrate re-activation */
  if (transfernumber < DMA_TRANSFER_COUNT) {
    DMA_RefreshPingPong(              /* Re-activate the DMA */
                        channel,
                        primary,
                        false,
                        (void *)&DMA_buffer[DMA_buffer_next][0],
                        NULL,
                        DMA_BUFFER_SIZE - 1,
                        false);
  } else {
    TIMER_Enable(TIMER0, false);      /* Stopping ADC by stopping TIMER0 */
    ADC_Reset(ADC0);                  /* Stopping ADC */
    transferActive = false;           /* Clearing Flag */
  } 
  
  /* do some calculations on the buffer with data ready for processing */
  averageValue[transfernumber-1]
    = calculateAverage(DMA_BUFFER_SIZE, &DMA_buffer[DMA_buffer_last][0]);

  //LEDoff();                           /* for debugging and timing check*/ 
  
}
/**************************************************************************//**
 * @brief  Enabling gpio Ports
 * @par
 *****************************************************************************/
void SetupGpio(void){
  
  GPIO_PinModeSet(ADC_Port, ADC_Pin, gpioModeInput,         1); /*Setup Pin of ADC*/  

  }

/**************************************************************************//**
 * @brief  Enabling clocks
 * @par
 *****************************************************************************/
void setupCmu(void) {
  /* Set HFRCO frequency */
  // CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);                                    /to be included by clocks.c
  CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
  // CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA,  true);  
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_PRS, true); 
  CMU_ClockEnable(cmuClock_HFPER, true);  /* enable peripheral clock */
  CMU_ClockEnable(cmuClock_GPIO, true);   /* enable GPIO clock */
}

/**************************************************************************//**
 * @brief Configure DMA for Ping-pong ADC to RAM Transfer
 * @par
 *****************************************************************************/
void setupDma(void) {
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  cbn.cbFunc  = transferComplete;
  cbn.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cbn;
  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_ADC, false, &descrCfg);
  
  /* Setting flag to indicate that transfer is in progress
    will be cleared by call-back function */
  transferActive = true;

  DMA_ActivatePingPong(               /* Enabling PingPong Transfer*/  
                       DMA_CHANNEL_ADC,
                       false,
                       (void *)&DMA_buffer[0][0],
                       (void *)&(ADC0->SINGLEDATA),
                       DMA_BUFFER_SIZE - 1,
                       (void *)&DMA_buffer[1][0],
                       (void *)&(ADC0->SINGLEDATA),
                       DMA_BUFFER_SIZE - 1);
}


/**************************************************************************//**
 * @brief Configure ADC to sample Vref/2 repeatedly at 10.5/13 Msamples/s
 * @par
 *****************************************************************************/
void setupAdc(void) {

  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;
  
  adcInit.ovsRateSel = adcOvsRateSel4; /* if adcInitSingle.resolution=adcResOVS */
  adcInit.lpfMode = adcLPFilterRC;
  adcInit.warmUpMode = adcWarmupNormal;
  //adcInit.warmUpMode = adcWarmupKeepADCWarm;
  //adcInit.warmUpMode = adcWarmupFastBG;
  //adcInit.timebase = _ADC_CTRL_TIMEBASE_DEFAULT;
  adcInit.timebase = 0;
  adcInit.prescale = ADC_PrescaleCalc(15*ADC_samp_freq, 0);
  //adcInit.prescale = _ADC_CTRL_PRESC_DEFAULT;
  //adcInit.prescale = 0;
  adcInit.tailgate = false;
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.prsSel = adcPRSSELCh0;      /* Triggered by PRS CH0 */
  adcInitSingle.acqTime = adcAcqTime1;
  adcInitSingle.reference = adcRefVDD;
  adcInitSingle.resolution = adcRes12Bit;   /* no oversampling */
  //adcInitSingle.resolution = adcResOVS;     /* oversampling */
  adcInitSingle.input = adcSingleInpCh0;    /* input = channel 0 */
  adcInitSingle.diff = false;
  adcInitSingle.prsEnable = true;			/* Enable PSR-trigger for ADC */
  adcInitSingle.leftAdjust = false;
  adcInitSingle.rep = false;
  ADC_InitSingle(ADC0, &adcInitSingle);
  
  /* Enable ADC single overflow interrupt to indicate lost samples */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLEOF);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Connect PRS channel 0 to TIMER overflow */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);
  
  /* Configure TIMER to trigger ADC sampling rate */
  TIMER_Init_TypeDef    timerInit     = TIMER_INIT_DEFAULT;
  TIMER_TopSet(TIMER0,  CMU_ClockFreqGet(cmuClock_TIMER0)/ADC_samp_freq);
  TIMER_Init(TIMER0, &timerInit);
}

 /**************************************************************************//**
 * @brief  Main function
 * @par
 *****************************************************************************/
//int main(void)
//{ 
  //CHIP_Init();    /* Initialize chip */												 //done by main
  
  //setupCmu();     /* Configuring clocks in the Clock Management Unit (CMU) */
  
  //setupLED();     /* Setup GPIO for LED */											//not needed
  
  //setupRC();      /* Setup GPIO for RC-lowpass-filter */                                //allready done by main
  
  //LEDon();        /* for debugging and timming */										//not needed
  
  //setupDma();     /* Configure DMA transfer from ADC to RAM using ping-pong */      	
  
  //setupAdc();     /* Configura ADC Sampling */
  
  //RChigh();       /* for debugging and timming */
  //LEDoff();       /* for debugging and timming */
  
  
  /* Wait in EM1 in until DMA is finished and callback is called */
  /* Disable interrupts until flag is checked in case DMA finishes after flag 
  * check but before sleep command. Device will still wake up on any set IRQ 
  * and any pending interrupts will be handled after interrupts are enabled 
  * again. */
  //INT_Disable();
  //while(transferActive) {
  //EMU_EnterEM1(); 
   //INT_Enable();
   //INT_Disable();
  //}
  //INT_Enable();
 
  //DMA_Reset();    /* Cleaning up after DMA transfers */

  //while (1);        /* Done */
//}
void InitADC()
{
  SetupGpio();    /* Setup the physical Gpio port for ADC*/ 
  
  setupCmu();     /* Configuring clocks in the Clock Management Unit (CMU) */

  setupDma();     /* Configure DMA transfer from ADC to RAM using ping-pong */      
  
  setupAdc();     /* Configura ADC Sampling */
  
  INT_Disable();  /* Disable interupt to hold CPU awake in EM1 */
}
void Measure()
{
  //transferComplete(unsigned int channel, bool primary, void *user);
  //DMA_Reset();
  while(transferActive) {
  //EMU_EnterEM1(); 
   INT_Enable();
   INT_Disable();
  }
  
  INT_Enable();  /* Activate interupt of DMA */
}
