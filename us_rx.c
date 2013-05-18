
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
#define ValBarrier                    3000      ///< ADC value barrier

/* ADC => DMA buffers and related */
DMA_CB_TypeDef cbn;                             ///< callback structure
volatile bool transferActive;                   ///< transfer flag
#define DMA_TRANSFER_COUNT            40        ///< x DMA transfers timeout for overdistance >60ms,10m
#define DMA_BUFFER_SIZE               256       ///< x entries in one DMA buffer, T-buffer 1,6ms
#define DMA_BUFFER_COUNT              4         ///< x DMA buffers >= 2
uint16_t DMA_buffer[DMA_BUFFER_COUNT][DMA_BUFFER_SIZE];   ///< the data
uint16_t DMA_buffer_last;                       ///< buffer ready for processing
uint16_t DMA_buffer_current;                    ///< buffer is currently filled
uint16_t DMA_buffer_next;                       ///< buffer will be used next

uint16_t averageValue[DMA_TRANSFER_COUNT];      ///< processed data
bool MaxReached = false;                        ///< stops ADC when Max reached
uint16_t MaxCount = 0;                          ///< gives back the time to reach max ADC
uint16_t transfernumber = 0;                    ///< number of transfered buffers

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
  while(1){
    /** ERROR: ADC Result overflow has occured.
     * This indicates that the DMA is not able to keep up with the ADC sample
     * rate and that a samples has been written to the ADC result registers
     * before the DMA was able to fetch the previous result. */ 
  }
}

/**************************************************************************//**
 * @brief  Comparing recived data to trigger measurements
 * @par
 *****************************************************************************/
void compareData(void)
{
    int i=0;									                  /*counts the value into the Buffer for checkup*/
	while ((i<DMA_BUFFER_SIZE)&&(MaxReached==false))
	{
		if ((uint16_t)ValBarrier < DMA_buffer[DMA_buffer_last][i]) {  /* Test value in DMA Buffer*/
			MaxCount = MaxCount+i;				            /* gives back complete count since Start measure*/
			MaxReached = true;				                /* Set Stopflag for ADC transfer*/
		}
		else
		{
			i++;
		}
	}	
	
}

/**************************************************************************//**
 * @brief  Call-back called when DMA transfer is complete
 * @param [in] DMA channel
 * @param [in] DMA descriptor: primary (true) or alternate (false)
 * @param [in] user pointer
 * @par
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  if( MaxReached )                            /* When maximum value */
  {
    TIMER_Enable(TIMER0, false);              /* Stopping ADC by stopping TIMER0 */
    transferActive = false;                   /* Clearing Flag */
  }
  
  else                                        /* Continiue when max not reaced*/
  {
    (void) user;

    MaxCount = transfernumber*256;            /* Calculate numbers of value in full buffers*/
    transfernumber++;                         /* Keeping track of the number of transfered buffers */


    
    /* numbers of DMA buffers in use */
    DMA_buffer_last = (transfernumber -1) % DMA_BUFFER_COUNT;
    DMA_buffer_current = (transfernumber) % DMA_BUFFER_COUNT;
    DMA_buffer_next = (transfernumber+1) % DMA_BUFFER_COUNT;
    
      
    /* Let the transfer be repeated a few times to illustrate re-activation */
    if (transfernumber < DMA_TRANSFER_COUNT)
    {
       DMA_RefreshPingPong(                   /* Re-activate the DMA */
                          channel,
                          primary,
                          false,
                          (void *)&DMA_buffer[DMA_buffer_next][0],
                          NULL,
                          DMA_BUFFER_SIZE - 1,
                          false);
    }
    else 
    {
      TIMER_Enable(TIMER0, false);            /* Stopping ADC by stopping TIMER0 */
      transferActive = false;                 /* Clearing Flag */
    } 
    
    if(transfernumber>2)                      /* Do not check Buffer 1 (overspeaking)*/
    {
      compareData();                          /* Check Buffer values for bursts*/
    }
  }
}


/**************************************************************************//**
 * @brief  Enabling gpio Ports
 * @par
 *****************************************************************************/
void SetupGpio(void) 
{  
  GPIO_PinModeSet(ADC_Port, ADC_Pin, gpioModeInput, 1); /*Setup Pin of ADC*/  
}

/**************************************************************************//**
 * @brief  Enabling clocks
 * @par
 *****************************************************************************/
void setupCmu(void) 
{
  /* Set HFRCO frequency */
  CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA,  true);   /* enable DMA clock */
  CMU_ClockEnable(cmuClock_ADC0, true);   /* enable ADC clock */
  CMU_ClockEnable(cmuClock_TIMER0, true); /* enable Timer0 clock */
  CMU_ClockEnable(cmuClock_PRS, true);    /* enable PRS clock */
  CMU_ClockEnable(cmuClock_HFPER, true);  /* enable peripheral clock */
  CMU_ClockEnable(cmuClock_GPIO, true);   /* enable GPIO clock */
  
}

/**************************************************************************//**
 * @brief Configure DMA for Ping-pong ADC to RAM Transfer
 * @par
 *****************************************************************************/
void setupDma(void)
{
  /* Setup config of DMA */
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

  DMA_ActivatePingPong(                   /* Enabling PingPong Transfer*/  
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
void setupAdc(void) 
{
  /* Setup config ADC */
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;
  
  adcInit.ovsRateSel = adcOvsRateSel4; /* if adcInitSingle.resolution=adcResOVS */
  adcInit.lpfMode = adcLPFilterRC;
  adcInit.warmUpMode = adcWarmupNormal;
  adcInit.timebase = 0;
  adcInit.prescale = ADC_PrescaleCalc(15*ADC_samp_freq, 0);
  adcInit.tailgate = false;
  
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.prsSel = adcPRSSELCh0;      /* Triggered by PRS CH0 */
  adcInitSingle.acqTime = adcAcqTime1;      /* Set aquision time */
  adcInitSingle.reference = adcRefVDD;      /* ADC reference Voltage usual 3.3V*/
  adcInitSingle.resolution = adcRes12Bit;   /* no oversampling */
  adcInitSingle.input = adcSingleInpCh0;    /* input = channel 0 */
  adcInitSingle.diff = false;               /* differential mode off */
  adcInitSingle.prsEnable = true;			      /* Enable PSR-trigger for ADC */
  adcInitSingle.leftAdjust = false;         /* adjusting false */
  adcInitSingle.rep = false;                /* repetitiv mode off */
  ADC_InitSingle(ADC0, &adcInitSingle);
  
  /* Enable ADC single overflow interrupt to indicate lost samples */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLEOF);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Connect PRS channel 0 to TIMER overflow */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);
  
  /* Configure TIMER to trigger ADC sampling rate */
  TIMER_Init_TypeDef    timerInit     = TIMER_INIT_DEFAULT;
  timerInit.enable = false;
  TIMER_TopSet(TIMER0,  CMU_ClockFreqGet(cmuClock_TIMER0)/ADC_samp_freq);
  TIMER_Init(TIMER0, &timerInit);
}

 /**************************************************************************//**
 * @brief  InitADC initial all ports, clocks, DMA, ADC routing and settings 
 * @par
 *****************************************************************************/
void InitADC(void)
{
  /* Setup the physical Gpio port for ADC*/
  SetupGpio();     
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  /* Configure DMA transfer from ADC to RAM using ping-pong */
  setupDma();           
  /* Configura ADC Sampling */
  setupAdc();     
  
}
 /**************************************************************************//**
 * @brief  Measure activate Timer0 and set values on default loop till peak or timeout
 * @par
 *****************************************************************************/
void Measure(void)
{
  /* Set variables on default */
  MaxReached = false;                         /* True in case of value higher than ValBarrier*/
  MaxCount = 0;                               /* counts the effective value of captured data*/
  transfernumber = 0;                         /* counts changes of buffers*/
  transferActive = true;                      /* False when transfernumber>DMA_TRANSFER_COUNT or MaxReached true*/

  /* Configure TIMER to trigger ADC sampling rate */  
  TIMER_Enable(TIMER0, true);                 /* Start ADC by starting TIMER0 */
  
  INT_Disable();                              /* Disable Interrupt till transferActive was tested */

  while(transferActive)                       /* enable/disable interrupt for reading ADC to memory*/
  {
   EMU_EnterEM1(); 
   INT_Enable();
   INT_Disable();
  }
  INT_Enable();                               /* Activate interupt of DMA */
  DMA_Reset();                                /* set back DMA for next use */
  
  SegmentLCD_Number(MaxCount);                /* Write value out on display */
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 0);     /* show Program State OFF */

}
