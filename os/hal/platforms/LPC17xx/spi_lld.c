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
 * @file    LPC17xx/spi_lld.c
 * @brief   LPC17xx low level SPI driver code.
 *
 * @addtogroup SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if LPC17xx_SPI_USE || defined(__DOXYGEN__)
/** @brief SPI1 driver identifier.*/
SPIDriver SPID1;
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
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_load_txdata(SPIDriver *spip) {

  LPC_SPI_TypeDef *spi = spip->spi;
  if (spip->txptr != NULL) {
    if ((spi->SPCR & SPCR_BIT_ENABLE)  || (!(spi->SPCR & SPCR_BIT_8))){
      const uint16_t *p = spip->txptr;
      spi->SPDR = *p++;
      spip->txptr = p;
    } else {
      const uint8_t *p = spip->txptr;
      spi->SPDR = *p++;
      spip->txptr = p;
    }
  } else {
    spi->SPDR = 0xffff;
  }
  spip->txcnt--;
}


/**
 * @brief   Common IRQ handler.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_serve_interrupt(SPIDriver *spip) {

  LPC_SPI_TypeDef *spi = spip->spi;

  /*
   * 数据读写溢出处理
   */
  if (spi->SPSR & (SPSR_ROVR | SPSR_WCOL)) {
    /* The overflow condition should never happen becausepriority is given
       to receive but a hook macro is provided anyway...*/
    LPC17xx_SPI_ERROR_HOOK(spip);
  }

  /*
   * 数据传输完成,启动下次传输任务
   */
  if (spi->SPSR & SPSR_SPIF) {
	  /* 
	   * 发送模式
	   */
	 if(spi->SPCR & SPCR_MSTR) {
		spi_load_txdata(spip); 
		if (spip->txcnt == 0) { 
			/*
			 * 清除中断 
			 */
			spi->SPINT = SPINT_CLS;
		}
	 }else{
	  /* 
	   * 接收模式
	   */
	   if (spip->rxptr != NULL) { 
		  if ((spi->SPCR & SPCR_BIT_ENABLE)  || (!(spi->SPCR & SPCR_BIT_8))){ 
			uint16_t *p = spip->rxptr; 
			*p++ = spi->SPDR; 
			spip->rxptr = p;
		  }else { 
			uint8_t *p = spip->rxptr; 
			*p++ = spi->SPDR; 
			spip->rxptr = p;
		  }
	   }else{ 
		  (void)spi->SPDR;
	   } 
	   if (--spip->rxcnt == 0) { 
		 chDbgAssert(spip->txcnt == 0,
                  "spi_serve_interrupt(), #1", "counter out of synch"); 
		 /* Stops the IRQ sources.*/ 
		 spi->SPINT = SPINT_CLS; 
		 /* Portable SPI ISR code defined in the high level driver, note, it is a macro.*/ 
		 _spi_isr_code(spip); 
		 return;
    	}
	 }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if LPC17xx_SPI_USE || defined(__DOXYGEN__)
/**
 * @brief   SSP interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(VectorF4) {

  CH_IRQ_PROLOGUE();

  spi_serve_interrupt(&SPID1);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
  spiObjectInit(&SPID1);
  LPC_PINCON->PINSEL0 |= (0x03ul << 30); 
  LPC_PINCON->PINSEL1 &= ~(0x03 << 0); 

  /* 设置SPI的片选引脚*/ 
  LPC_GPIO0->FIODIR   |=  SPI_CS;                        

  /* 设置P0.17、P0.18 */
  LPC_PINCON->PINSEL1 |=  (0x03 << 2) | (0x03 << 4);    
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {
  spip->spi->SPCR = spip->config->spcr;
  spip->spi->SPCCR = spip->config->spccr;

  LPC_SC->PCONP |= (1UL << 8);
  nvicEnableVector(SPI_IRQn,
                       CORTEX_PRIORITY_MASK(LPC17xx_SPI_IRQ_PRIORITY));

}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state != SPI_STOP) {
	  spip->spi->SPCR = 0;
	  spip->spi->SPCCR = 0; 
  }
  LPC_SC->PCONP &= ~(1UL << 8);
  nvicDisableVector(SPI_IRQn);
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  palClearPad(spip->config->spiport, spip->config->spiad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  palSetPad(spip->config->spiport, spip->config->spiad);
}



/**
 * @brief   Ignores data on the SPI bus.
 * @details This function transmits a series of idle words on the SPI bus and
 *          ignores the received data. This function can be invoked even
 *          when a slave select signal has not been yet asserted.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spi_lld_exchange( spip, n, NULL, NULL);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->rxptr = rxbuf;
  spip->txptr = txbuf;
  spip->rxcnt = spip->txcnt = n;

  spi_load_txdata(spip);
  spip->spi->SPCR = SPCR_SPIE;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {
  spi_lld_exchange( spip, n, txbuf, NULL);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {
  spi_lld_exchange( spip, n, NULL, rxbuf);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spip->spi->SPDR = frame;
  while ((spip->spi->SPSR & SPSR_SPIF) == 0)
    ;
  return (uint16_t)spip->spi->SPDR;
}

#endif /* HAL_USE_SPI */

/** @} */
