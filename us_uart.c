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
#include "em_device.h"
#include "dmactrl.h"
#include "em_leuart.h"
#include "em_dma.h"

#include "globals.h"
#include "us_rx.h"





/*****************************************************************************
 * data definitions
 *****************************************************************************/
 /* DEFINES */

#define DMA_CHANNEL           0
#define BUF_MAX               2*DMA_BUF_SIZE // was 8 for hello 


/* DMA callback structure */
DMA_CB_TypeDef cb[DMA_CHAN_COUNT];

/* Defining the LEUART0 initialization data */
LEUART_Init_TypeDef leuart0Init =
{
  .enable   = leuartEnableTx,           /* Activate data reception on LEUn_TX pin. */
  .refFreq  = 0,                        /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = 115200,                   /* Baudrate = 115200 bps =>8,68us */
  .databits = leuartDatabits8,          /* Each LEUART frame containes 8 databits */
  .parity   = leuartNoParity,           /* No parity bits in use */
  .stopbits = leuartStopbits1,          /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/* DMA init structure */
DMA_Init_TypeDef dmaInit = 
{
  .hprot        = 0,                    /* No descriptor protection */
  .controlBlock = dmaControlBlock,      /* DMA control block alligned to 256 */
};

/* Setting up DMA channel */
DMA_CfgChannel_TypeDef chnlCfg =
{
  .highPri   = false,                   /* Normal priority */
  .enableInt = false,                   /* No interupt for callback function */
  .select    = DMAREQ_LEUART0_TXBL,     /* Set LEUART0 TX buffer empty, as source of DMA signals */
  .cb        = &(cb[DMA_CHANNEL]),      /* Callback */
};

/* Setting up channel descriptor */
DMA_CfgDescr_TypeDef descrCfg =
{
  .dstInc  = dmaDataIncNone,            /* Do not increment destination address */
  .srcInc  = dmaDataInc1,               /* Increment source address by one byte */
  .size    = dmaDataSize1,              /* Data size is one byte */
  .arbRate = dmaArbitrate1,             /* Rearbitrate for each byte recieved */
  .hprot   = 0,                         /* No read/write source protection */
};

/**************************************************************************//**
 * @brief LeuartSend
 *
 * This routine will run on every call up and send the DMA data over Uart
 *
 *****************************************************************************/ 
void leuartsend(void)
{
  int i=0;
  if(i==0)
  {
    /* Set new DMA source address directly in the DMA descriptor */
    dmaControlBlock->SRCEND = (char*)DMA_buf[DMA_buf_last] + BUF_MAX - 1; /*Set Pointer to measured data in memory */
    i=1;
  }
  else
  {
    /* Set new DMA source address directly in the DMA descriptor */
    dmaControlBlock->SRCEND = (char*)DMA_buf[DMA_buf_current] + BUF_MAX - 1; /*Set Pointer to measured data in memory */
    i=0;
  }
  /* Enable DMA wake-up from LEUART0 TX */
  LEUART0->CTRL = LEUART_CTRL_TXDMAWU;

  /* (Re)starting the transfer. Using Basic Mode */
  DMA_ActivateBasic(DMA_CHANNEL,           /* Activate channel selected */
                    true,                  /* Use primary descriptor */
                    false,                 /* No DMA burst */
                    NULL,                  /* Keep destination address */
                    NULL,                  /* Keep source address*/
                    BUF_MAX - 1);          /* Size of buffer minus1 */

 

}

  
/**************************************************************************//**
 * @brief  DMA Callback function
 *
 * When the DMA transfer is completed, disables the DMA wake-up on TX in the 
 * LEUART to enable the DMA to sleep even when the LEUART buffer is empty.
 *
 ******************************************************************************/
void dmaTransferDone(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Disable DMA wake-up from LEUART0 TX */
  LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;
}


/**************************************************************************//**
 * @brief  Initialize Low Energy UART 0
 *
 * Here the LEUART is initialized with the chosen settings. It is then routed
 * to location 0 to avoid conflict with the LCD pinout. Finally the GPIO mode
 * is set to push pull.
 *
 *****************************************************************************/
void initLeuart(void)
{
  /* Reseting and initializing LEUART0 */
  LEUART_Reset(LEUART0);
  LEUART_Init(LEUART0, &leuart0Init);

  /* Route LEUART0 TX pin to DMA location 0 */
  LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART0. TX is on D6 */
  GPIO_PinModeSet(gpioPortD,                /* GPIO port */
                  4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */
}


/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. The destination for all the DMA transfers through this
 * channel is set to be the LEUART0 TXDATA register, and transfer complete
 * interrupt is enabled.
 *
 *****************************************************************************/
void setupLeuartDma(void)
{
  /* Setting call-back function */  
  cb[DMA_CHANNEL].cbFunc  = dmaTransferDone;
  cb[DMA_CHANNEL].userPtr = NULL;
  
  /* Initializing DMA, channel and desriptor */
  DMA_Init(&dmaInit);  
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);

  /* Set new DMA destination address directly in the DMA descriptor */
  dmaControlBlock->DSTEND = &LEUART0->TXDATA;

  /* Enable DMA Transfer Complete Interrupt */
  DMA->IEN = DMA_IEN_CH0DONE;

  /* Enable DMA interrupt vector */
  NVIC_EnableIRQ(DMA_IRQn);
}


/**************************************************************************//**
 * @brief  InitUart
 *
 * Setup clock routing and enable clocks, Route and activate Pins setup DMA device
 *
 *****************************************************************************/

void InitUart(void)
{
  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
  CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_2 );
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Enabling clocks, all other remain disabled */
  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
  CMU_ClockEnable(cmuClock_CORE, true);       /* Enable CORE clock*/
  CMU_ClockEnable(cmuClock_HFPER, true);      /* Enable HF Peripheral*/
  CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */   //done by Main?
  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART0 */
  initLeuart();                               /* setup Uart IO*/ 
  setupLeuartDma();                           /* setup Uart DMA*/

}
void SendUart(void)
{


  leuartsend();

}
