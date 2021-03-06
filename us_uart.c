/**************************************************************************//**
 * @file us_uart.c
 *
 * @brief Serial interface 
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-04-30
 * @version 0.1
 *
 * @verbatim
 * Sending receiving data over serial interface
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
#include "em_device.h"
#include "dmactrl.h"
#include "em_leuart.h"
#include "em_dma.h"

#include "globals.h"
#include "us_rx.h"



/*******************************************************************************
 **************************   DATA DEFINITIONS   *******************************
 ******************************************************************************/



/*****************************************************************************
 * data definitions
 *****************************************************************************/
 /* DEFINES */

#define DMA_CHANNEL           0
#define BUF_MAX               2*DMA_BUF_SIZE // was 8 for hello 

uint16_t  uart_buf_sent = 0;
uint16_t  buf_to_send = 0;

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



/*******************************************************************************
 **************************   LOCAL FUNCTIONS   ********************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief LeuartSend
 *
 * This routine will run on every call up and send the DMA data over Uart
 *
 *****************************************************************************/ 
void leuartSend(void)
{
  uart_buf_sent = 0;
  
  if ( MaxCount > 3*DMA_BUF_SIZE )
  {
    buf_to_send = (MaxCount/DMA_BUF_SIZE - 1) % DMA_BUFS;
  }
  else
  {
    buf_to_send = 0;
  }
  /* Set new DMA source address directly in the DMA descriptor */
  dmaControlBlock->SRCEND = (char*)DMA_buf[buf_to_send] + BUF_MAX - 1; /*Set Pointer to measured data in memory */
  
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
  (void) user;
  if (uart_buf_sent < 2)
  {
    uart_buf_sent++;
    buf_to_send = (buf_to_send + 1) % DMA_BUFS;
    /* Set new DMA source address directly in the DMA descriptor */
    dmaControlBlock->SRCEND = (char*)DMA_buf[buf_to_send] + BUF_MAX - 1; /*Set Pointer to measured data in memory */
  
    /* (Re)starting the transfer. Using Basic Mode */
    DMA_ActivateBasic(channel,           /* Activate channel selected */
                      primary,                  /* Use primary descriptor */
                      false,                 /* No DMA burst */
                      NULL,                  /* Keep destination address */
                      NULL,                  /* Keep source address*/
                      BUF_MAX - 1);          /* Size of buffer minus1 */
  }
  else
  {
    (void) channel;
    (void) primary;
    
    /* Disable DMA wake-up from LEUART0 TX */
    LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;
  }
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
  LEUART_Reset( LEUART0 );
  LEUART_Init ( LEUART0, &leuart0Init );

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
  DMA_Init( &dmaInit );  
  DMA_CfgChannel( DMA_CHANNEL, &chnlCfg );
  DMA_CfgDescr  ( DMA_CHANNEL, true, &descrCfg );

  /* Set new DMA destination address directly in the DMA descriptor */
  dmaControlBlock->DSTEND = &LEUART0->TXDATA;

  /* Enable DMA Transfer Complete Interrupt */
  DMA->IEN = DMA_IEN_CH0DONE;

  /* Enable DMA interrupt vector */
  NVIC_EnableIRQ( DMA_IRQn );
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
  CMU_ClockSelectSet( cmuClock_LFB,     cmuSelect_CORELEDIV2 );
  CMU_ClockDivSet   ( cmuClock_LEUART0, cmuClkDiv_2 );
  CMU_ClockSelectSet( cmuClock_HF,      cmuSelect_HFXO );

  /* Enabling clocks, all other remain disabled */
  CMU_ClockEnable( cmuClock_CORELE,  true );  /* Enable CORELE clock */
  CMU_ClockEnable( cmuClock_CORE,    true );  /* Enable CORE clock*/
  CMU_ClockEnable( cmuClock_HFPER,   true );  /* Enable HF Peripheral*/
  CMU_ClockEnable( cmuClock_DMA,     true );  /* Enable DMA clock */
  CMU_ClockEnable( cmuClock_GPIO,    true );  /* Enable GPIO clock */   //done by Main?
  CMU_ClockEnable( cmuClock_LEUART0, true );  /* Enable LEUART0 */
  
  initLeuart();                               /* setup Uart IO*/ 
  setupLeuartDma();                           /* setup Uart DMA*/

}



/**************************************************************************//**
 * @brief  SendUart
 *
 * Setup clock routing and enable clocks, Route and activate Pins setup DMA device
 *
 *****************************************************************************/

void SendUart(void)
{

  leuartSend();

}
