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
 * @file    LPC17xx/spi_lld.h
 * @brief   LPC17xx low level SPI driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI_LLD_H_
#define _SPI_LLD_H_

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


#define SPCR_BIT_ENABLE 		4
#define SPCR_CPHA				8
#define SPCR_CPOL				(0x0001<<4)
#define SPCR_MSTR				(0x0001<<5)
#define SPCR_LSBF				(0x0001<<6)
#define SPCR_SPIE 				(0x0001<<7)
#define SPCR_BIT_8				(0x0008<<7)
#define SPCR_BIT_9				(0x0009<<7)
#define SPCR_BIT_10				(0x000A<<7)
#define SPCR_BIT_11				(0x000B<<7)
#define SPCR_BIT_12				(0x000C<<7)
#define SPCR_BIT_13				(0x000D<<7)
#define SPCR_BIT_14				(0x000E<<7)
#define SPCR_BIT_15				(0x000F<<7)
#define SPCR_BIT_16				(0x0000<<7)

#define SPSR_ABRT 				0x08
#define SPSR_MODF 				0x10
#define SPSR_ROVR 				0x20
#define SPSR_WCOL				0x40
#define SPSR_SPIF 				0x80

#define SPINT_CLS				0x01


/* 
 * P0.16¿ÚÎªÆ¬Ñ¡½Å   
 */
#define    SPI_CS      (1 << 16)         

/**
 * @brief   SCK0 signal assigned to pin PIO0_10.
 */
#define SCK0_IS_PIO0_10         0

/**
 * @brief   SCK0 signal assigned to pin PIO2_11.
 */
#define SCK0_IS_PIO2_11         1

/**
 * @brief   SCK0 signal assigned to pin PIO0_6.
 */
#define SCK0_IS_PIO0_6          2

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for device SSP0 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LPC17xx_SPI_USE) || defined(__DOXYGEN__)
#define LPC17xx_SPI_USE						TRUE
#endif


/**
 * @brief   SPI0 interrupt priority level setting.
 */
#if !defined(LPC17xx_SPI_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_SPI_IRQ_PRIORITY       		5
#endif


/**
 * @brief   Overflow error hook.
 * @details The default action is to stop the system.
 */
#if !defined(LPC17xx_SPI_ERROR_HOOK) || defined(__DOXYGEN__)
#define LPC17xx_SPI_ERROR_HOOK(spip)    chSysHalt()
#endif

/**
 * @brief   SCK0 signal selector.
 */
#if !defined(LPC17xx_SPI_SCK0_SELECTOR) || defined(__DOXYGEN__)
#define LPC17xx_SPI_SCK0_SELECTOR           SCK0_IS_PIO2_11
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/


/**
 * @brief   SSP0 clock.
 */
#define LPC17xx_SPI_SSP0_PCLK                                               \
  (LPC17xx_MAINCLK / LPC17xx_SPI_SSP0CLKDIV)

/**
 * @brief   SSP1 clock.
 */
#define LPC17xx_SPI_SSP1_PCLK                                               \
  (LPC17xx_MAINCLK / LPC17xx_SPI_SSP1CLKDIV)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SPIDriver SPIDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spip      pointer to the @p SPIDriver object triggering the
 *                      callback
 */
typedef void (*spicallback_t)(SPIDriver *spip);

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief Operation complete callback or @p NULL.
   */
  spicallback_t         end_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief The chip select line port.
   */
  ioportid_t 			spiport;
  /**
   * @brief The chip select line pad number.
   */
  uint16_t              spiad;

  /**
   * @brief SPI SPCCR initialization data.
   */
  uint16_t              spccr;

  /**
   * @brief SPI SPCR initialization data.
   */
  uint32_t              spcr;
} SPIConfig;

/**
 * @brief   Structure representing a SPI driver.
 */
struct SPIDriver {
  /**
   * @brief Driver state.
   */
  spistate_t            state;
  /**
   * @brief Current configuration data.
   */
  const SPIConfig       *config;
#if SPI_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  Thread                *thread;
#endif /* SPI_USE_WAIT */
#if SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the bus.
   */
  Mutex                 mutex;
#elif CH_USE_SEMAPHORES
  Semaphore             semaphore;
#endif
#endif /* SPI_USE_MUTUAL_EXCLUSION */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the SSP registers block.
   */
  //LPC_SSP_TypeDef       *ssp;
  LPC_SPI_TypeDef 		*spi;
  /**
   * @brief Number of bytes yet to be received.
   */
  uint32_t              rxcnt;
  /**
   * @brief Receive pointer or @p NULL.
   */
  void                  *rxptr;
  /**
   * @brief Number of bytes yet to be transmitted.
   */
  uint32_t              txcnt;
  /**
   * @brief Transmit pointer or @p NULL.
   */
  const void            *txptr;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if LPC17xx_SPI_USE && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif


#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
  void spi_lld_ignore(SPIDriver *spip, size_t n);
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI_LLD_H_ */

/** @} */
