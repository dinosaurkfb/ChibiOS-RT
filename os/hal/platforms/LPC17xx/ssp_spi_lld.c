/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    LPC17xx/ssp_spi_lld.c
 * @brief   LPC18xx low level SSP_SPI driver code.
 *
 * @addtogroup SSP_SPI
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "pinsel_lld.h"
#include "lpc177x_8x_gpio.h"

#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if LPC17xx_USE_SSP_SPI0 || defined(__DOXYGEN__)
/** @brief SPI1 driver identifier.*/
SSPSPIDriver SPID1;
#endif

#if LPC17xx_USE_SSP_SPI1 || defined(__DOXYGEN__)
/** @brief SPI2 driver identifier.*/
SSPSPIDriver SPID2;
#endif

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2 || defined(__DOXYGEN__)
/** @brief SPI2 driver identifier.*/
SSPSPIDriver SPID3;
#endif
#endif


/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Preloads the transmit FIFO.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 */
static void ssp_fifo_preload(SSPSPIDriver *spip) {
  LPC_SSP_TypeDef *ssp = spip->ssp;
  uint32_t n = spip->txcnt > LPC17xx_SSP_FIFO_DEPTH ?
               LPC17xx_SSP_FIFO_DEPTH : spip->txcnt;

  while(((ssp->SR & SR_TNF) != 0) && (n > 0)) {
    if (spip->txptr != NULL) {
      if ((ssp->CR0 & CR0_DSSMASK) > CR0_DSS8BIT) {
        const uint16_t *p = spip->txptr;
        ssp->DR = *p++;
        spip->txptr = p;
      }
      else {
        const uint8_t *p = spip->txptr;
        ssp->DR = *p++;
        spip->txptr = p;
      }
    }
    else
      ssp->DR = 0xFFFFFFFF;
    n--;
    spip->txcnt--;
  }
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 */
static void ssp_spi_serve_interrupt(SSPSPIDriver *spip) {
  LPC_SSP_TypeDef *ssp = spip->ssp;

  if ((ssp->MIS & MIS_ROR) != 0) {
    /* The overflow condition should never happen because priority is given
       to receive but a hook macro is provided anyway...*/
    LPC17xx_SSP_SPI_ERROR_HOOK(spip);
  }
  ssp->ICR = ICR_RT | ICR_ROR;
  while ((ssp->SR & SR_RNE) != 0) {
    if (spip->rxptr != NULL) {
      if ((ssp->CR0 & CR0_DSSMASK) > CR0_DSS8BIT) {
        uint16_t *p = spip->rxptr;
        *p++ = ssp->DR;
        spip->rxptr = p;
      }
      else {
        uint8_t *p = spip->rxptr;
        *p++ = ssp->DR;
        spip->rxptr = p;
      }
    }
    else
      (void)ssp->DR;
    if (--spip->rxcnt == 0) {
      chDbgAssert(spip->txcnt == 0,
                  "ssp_spi_serve_interrupt(), #1", "counter out of synch");
      /* Stops the IRQ sources.*/
      ssp->IMSC = 0;
      /* Portable SPI ISR code defined in the high level driver, note, it is
         a macro.*/
      _spi_isr_code(spip);
      return;
    }
  }
  ssp_fifo_preload(spip);
  if (spip->txcnt == 0)
    ssp->IMSC = IMSC_ROR | IMSC_RT | IMSC_RX;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if LPC17xx_USE_SSP_SPI0 || defined(__DOXYGEN__)
/**
 * @brief   SSP0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector78) {

  CH_IRQ_PROLOGUE();

  ssp_spi_serve_interrupt(&SPID1);

  CH_IRQ_EPILOGUE();
}
#endif

#if LPC17xx_USE_SSP_SPI1 || defined(__DOXYGEN__)
/**
 * @brief   SSP1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector7C) {

  CH_IRQ_PROLOGUE();

  ssp_spi_serve_interrupt(&SPID2);

  CH_IRQ_EPILOGUE();
}
#endif

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2 || defined(__DOXYGEN__)
/**
 * @brief   SSP1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(VectorD0) {

  CH_IRQ_PROLOGUE();

  ssp_spi_serve_interrupt(&SPID3);

  CH_IRQ_EPILOGUE();
}
#endif
#endif

static void set_ssp_spi_clock(SSPSPIDriver *spip, uint32_t target_clock)
{
    uint32_t prescale, cr0_div, cmp_clk, ssp_clk;
	ssp_clk = LPC17xx_PCLK;

	/* Find closest divider to get at or under the target frequency.
	   Use smallest prescale possible and rely on the divider to get
	   the closest target frequency */
	cr0_div = 0;
	cmp_clk = 0xFFFFFFFF;
	prescale = 2;
	while (cmp_clk > target_clock)
	{
		cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
		if (cmp_clk > target_clock)
		{
			cr0_div++;
			if (cr0_div > 0xFF)
			{
				cr0_div = 0;
				prescale += 2;
			}
		}
	}

    /* Write computed prescaler and divider back to register */
    spip->ssp->CR0 &= (~CR0_SCR(0xFF)) & CR0_BITMASK;
    spip->ssp->CR0 |= (CR0_SCR(cr0_div)) & CR0_BITMASK;
    spip->ssp->CPSR = prescale & 0xFF;
}



/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void ssp_spi_lld_init(void) {

#if LPC17xx_USE_SSP_SPI0
  sspspiObjectInit(&SPID1);
  SPID1.ssp = LPC_SSP0; 
#endif /* LPC17xx_USE_SSP_SPI0 */

#if LPC17xx_USE_SSP_SPI1
  sspspiObjectInit(&SPID2);
  SPID2.ssp = LPC_SSP1;
#endif /* LPC17xx_USE_SSP_SPI0 */

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2
  sspspiObjectInit(&SPID3);
  SPID3.ssp = LPC_SSP2;
#endif /* LPC17xx_USE_SSP_SPI2 */
#endif 
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
void ssp_spi_lld_start(SSPSPIDriver *spip) {

  if (spip->state == SSP_SPI_STOP) {
    /* Clock activation.*/
#if LPC17xx_USE_SSP_SPI0

#ifdef LPC177x_8x
	/*
	* Initialize SPI pin connect
	* P2.19 - SSEL - used as GPIO
	* P2.22 - SCK
	* P2.26 - MISO
	* P2.27 - MOSI
	*/
	PINSEL_ConfigPin(2, 22, 2);	 /* SSP0_SCK */
	PINSEL_ConfigPin(2, 26, 2);	 /* SSP0_MISO */
	PINSEL_ConfigPin(2, 27, 2);	 /* SSP0_MOSI */

	/* P2.19 CS is output */
	PINSEL_ConfigPin(2, 19, 0);	 /* P2.19 - GPIO */
    GPIO_SetDir(2, (1<<19), 1);

#else
	LPC_GPIO0->FIODIR |=  (1<<16);                   /* P0.16 CS is output */
	/* P0.15 SCK, P0.17 MISO, P0.18 MOSI are SSP pins. */
	LPC_PINCON->PINSEL0 &= ~( (2UL<<30) ); 		     /* P0.15  cleared */
	LPC_PINCON->PINSEL1 &= ~( (2<<2) | (2<<4) );     /* P0.17, P0.18  cleared */

	LPC_PINCON->PINSEL0 |=  ( 2UL<<30);              /* P0.15 SCK0 */
	LPC_PINCON->PINSEL1 |=  ( 2<<2) | (2<<4) ;       /* P0.17 MISO0   P0.18 MOSI0 */
#endif
    if (&SPID1 == spip) { 
	  LPC_SC->PCONP |= PCSSP0;                      /* Enable power to SSPI0 block */ 
      nvicEnableVector(SSP0_IRQn,
                       CORTEX_PRIORITY_MASK(LPC17xx_SSP_SPI0_IRQ_PRIORITY));
    }
#endif

#if LPC17xx_USE_SSP_SPI1
#ifdef LPC177x_8x
	/*
	* Initialize SPI pin connect
	* P5.4 -  SSEL - used as GPIO
	* P1.31 - SCK
	* P1.18 - MISO
	* P0.13 - MOSI
	*/
	/* Default P5.4 - GPIO */
	PINSEL_ConfigPin(1, 31, 2);	 /* SSP1_SCK */
	PINSEL_ConfigPin(1, 18, 5);	 /* SSP1_MISO */
	PINSEL_ConfigPin(0, 13, 2);  /* SSP1_MOSI */

    /* P5.4 CS is output */
	LPC_GPIO5->FIODIR |= 1 << 4;
#else
	/* Please Configure PIN (CS SCK MISO MOSI) */
//	LPC_GPIO0->FIODIR |=  (1<<6);                   /* P0.6 CS is output */

	/* P0.7 SCK, P0.8 MISO, P0.9 MOSI are SSP pins. */
	LPC_PINCON->PINSEL0 &=~((3<<14) | (3<<16) | (3<<18));
	LPC_PINCON->PINSEL0 |= (2<<14)|(2<<16)|(2<<18);    

	LPC_SC->PCLKSEL0 &=~(3<<20);                   
	LPC_SC->PCLKSEL0 |=(1<<20);                   
#endif
    if (&SPID2 == spip) {
	  LPC_SC->PCONP |= PCSSP1;                      /* Enable power to SSPI1 block */ 
      nvicEnableVector(SSP1_IRQn,
                       CORTEX_PRIORITY_MASK(LPC17xx_SSP_SPI1_IRQ_PRIORITY));
    }
#endif

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2
	/* Please Configure PIN (CS SCK MISO MOSI) */
	// ----------
	// ----------
	/* */
    if (&SPID3 == spip) {
	  LPC_SC->PCONP |= PCSSP2;                      /* Enable power to SSPI2 block */ 
      nvicEnableVector(SSP1_IRQn,
                       CORTEX_PRIORITY_MASK(LPC17xx_SSP_SPI2_IRQ_PRIORITY));
    }
#endif
#endif
  }

#ifdef LPC177x_8x
#else
  /* PCLK_SSP0=CCLK */
  LPC_SC->PCLKSEL1 &= ~(3<<10);                    /* PCLKSP0 = CCLK/4 (18MHz) */
  LPC_SC->PCLKSEL1 |=  (1<<10);                    /* PCLKSP0 = CCLK   (72MHz) */
#endif

  /* Configuration.*/
  spip->ssp->CR1  = 0;
  spip->ssp->ICR  = ICR_RT | ICR_ROR;
  spip->ssp->CR0  = spip->config->cr0;
  set_ssp_spi_clock(spip, spip->config->bit_rate);
  spip->ssp->CR1  = CR1_SSE;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
void ssp_spi_lld_stop(SSPSPIDriver *spip) {

  if (spip->state != SSP_SPI_STOP) {
    spip->ssp->CR1  = 0;
    spip->ssp->CR0  = 0;
    spip->ssp->CPSR = 0;
#if LPC17xx_USE_SSP_SPI0
    if (&SPID1 == spip) {
	  LPC_SC->PCONP |= PCSSP0;                   
      nvicDisableVector(SSP0_IRQn);
    }
#endif
#if LPC17xx_USE_SSP_SPI1
    if (&SPID2 == spip) {
	  LPC_SC->PCONP |= PCSSP1;                   
      nvicDisableVector(SSP1_IRQn);
    }
#endif

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2
    if (&SPID3 == spip) {
	  LPC_SC->PCONP |= PCSSP2;                   
      nvicDisableVector(SSP2_IRQn);
    }
#endif
#endif
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
void ssp_spi_lld_select(SSPSPIDriver *spip) {

  palClearPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
void ssp_spi_lld_unselect(SSPSPIDriver *spip) {

  palSetPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This function transmits a series of idle words on the SPI bus and
 *          ignores the received data. This function can be invoked even
 *          when a slave select signal has not been yet asserted.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void ssp_spi_lld_ignore(SSPSPIDriver *spip, size_t n) {

  spip->rxptr = NULL;
  spip->txptr = NULL;
  spip->rxcnt = spip->txcnt = n;
  ssp_fifo_preload(spip);
  spip->ssp->IMSC = IMSC_ROR | IMSC_RT | IMSC_TX | IMSC_RX;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void ssp_spi_lld_exchange(SSPSPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->rxptr = rxbuf;
  spip->txptr = txbuf;
  spip->rxcnt = spip->txcnt = n;
  ssp_fifo_preload(spip);
  spip->ssp->IMSC = IMSC_ROR | IMSC_RT | IMSC_TX | IMSC_RX;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void ssp_spi_lld_send(SSPSPIDriver *spip, size_t n, const void *txbuf) {

  spip->rxptr = NULL;
  spip->txptr = txbuf;
  spip->rxcnt = spip->txcnt = n;
  ssp_fifo_preload(spip);
  spip->ssp->IMSC = IMSC_ROR | IMSC_RT | IMSC_TX | IMSC_RX;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void ssp_spi_lld_receive(SSPSPIDriver *spip, size_t n, void *rxbuf) {

  spip->rxptr = rxbuf;
  spip->txptr = NULL;
  spip->rxcnt = spip->txcnt = n;
  ssp_fifo_preload(spip);
  spip->ssp->IMSC = IMSC_ROR | IMSC_RT | IMSC_TX | IMSC_RX;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t ssp_spi_lld_polled_exchange(SSPSPIDriver *spip, uint16_t frame) {

  spip->ssp->DR = (uint32_t)frame;
  while ((spip->ssp->SR & SR_RNE) == 0)
    ;
  return (uint16_t)spip->ssp->DR;
}

#endif /* HAL_USE_SPI */

/** @} */
