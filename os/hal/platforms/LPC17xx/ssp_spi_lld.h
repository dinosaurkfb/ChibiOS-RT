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
 * @file    LPC17xx/ssp_spi_lld.h
 * @brief   LPC17xx low level SSP_SPI driver header.
 *
 * @addtogroup SSP_SPI
 * @{
 */

#ifndef _ssp_spi_lld_H_
#define _ssp_spi_lld_H_

#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define PCSSP0 					(((uint32_t)1)<<21)
#define PCSSP1 					(((uint32_t)1)<<10)

#ifdef LPC177x_8x
#define PCSSP2 					(((uint32_t)1)<<20)
#endif
/**
 * @brief   Hardware FIFO depth.
 */
#define LPC17xx_SSP_FIFO_DEPTH  8

#define CR0_DSSMASK             0x0F
#define CR0_DSS4BIT             3
#define CR0_DSS5BIT             4
#define CR0_DSS6BIT             5
#define CR0_DSS7BIT             6
#define CR0_DSS8BIT             7
#define CR0_DSS9BIT             8
#define CR0_DSS10BIT            9
#define CR0_DSS11BIT            0xA
#define CR0_DSS12BIT            0xB
#define CR0_DSS13BIT            0xC
#define CR0_DSS14BIT            0xD
#define CR0_DSS15BIT            0xE
#define CR0_DSS16BIT            0xF
#define CR0_FRFSPI              0
#define CR0_FRFSSI              0x10
#define CR0_FRFMW               0x20
#define CR0_CPOL                0x40
#define CR0_CPHA                0x80
/** SSP serial clock rate value load macro, divider rate is PERIPH_CLK / (cpsr * (SCR + 1)) */
#define CR0_SCR(n)   			((uint32_t)((n&0xFF)<<8))
#define CR0_BITMASK				((uint32_t)(0xFFFF))


#define CR1_LBM                 1
#define CR1_SSE                 2
#define CR1_MS                  4
#define CR1_SOD                 8

#define SR_TFE                  1
#define SR_TNF                  2
#define SR_RNE                  4
#define SR_RFF                  8
#define SR_BSY                  0x10

#define IMSC_ROR                1
#define IMSC_RT                 2
#define IMSC_RX                 4
#define IMSC_TX                 8

#define RIS_ROR                 1
#define RIS_RT                  2
#define RIS_RX                  4
#define RIS_TX                  8

#define MIS_ROR                 1
#define MIS_RT                  2
#define MIS_RX                  4
#define MIS_TX                  8

#define ICR_ROR                 1
#define ICR_RT                  2

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
#if !defined(LPC17xx_USE_SSP_SPI0) || defined(__DOXYGEN__)
#define LPC17xx_USE_SSP_SPI0                TRUE
#endif

/**
 * @brief   SPI2 driver enable switch.
 * @details If set to @p TRUE the support for device SSP1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LPC17xx_USE_SSP_SPI1) || defined(__DOXYGEN__)
#define LPC17xx_USE_SSP_SPI1                FALSE
#endif

/**
 * @brief   SSP0 PCLK divider.
 */
#if !defined(LPC17xx_SSP_SPI0CLKDIV) || defined(__DOXYGEN__)
#define LPC17xx_SSP_SPI0CLKDIV              1
#endif

/**
 * @brief   SSP1 PCLK divider.
 */
#if !defined(LPC17xx_SSP_SPI1CLKDIV) || defined(__DOXYGEN__)
#define LPC17xx_SSP_SPI1CLKDIV              1
#endif

/**
 * @brief   SPI0 interrupt priority level setting.
 */
#if !defined(LPC17xx_SSP_SPI0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_SSP_SPI0_IRQ_PRIORITY       5
#endif

/**
 * @brief   SPI1 interrupt priority level setting.
 */
#if !defined(LPC17xx_SSP_SPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_SSP_SPI1_IRQ_PRIORITY       5
#endif

/**
 * @brief   Overflow error hook.
 * @details The default action is to stop the system.
 */
#if !defined(LPC17xx_SSP_SPI_ERROR_HOOK) || defined(__DOXYGEN__)
#define LPC17xx_SSP_SPI_ERROR_HOOK(spip)    chSysHalt()
#endif

/**
 * @brief   SCK0 signal selector.
 */
#if !defined(LPC13xx_SPI_SCK0_SELECTOR) || defined(__DOXYGEN__)
#define LPC13xx_SPI_SCK0_SELECTOR           SCK0_IS_PIO2_11
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (LPC17xx_SSP_SPI0CLKDIV < 1) || (LPC17xx_SSP_SPI0CLKDIV > 255)
#error "invalid LPC17xx_SSP_SPI0CLKDIV setting"
#endif

#if (LPC17xx_SSP_SPI1CLKDIV < 1) || (LPC17xx_SSP_SPI1CLKDIV > 255)
#error "invalid LPC17xx_SSP_SPI1CLKDIV setting"
#endif

#if !LPC17xx_USE_SSP_SPI0 && !LPC17xx_USE_SSP_SPI1
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if (LPC13xx_SPI_SCK0_SELECTOR != SCK0_IS_PIO0_10) &&                       \
    (LPC13xx_SPI_SCK0_SELECTOR != SCK0_IS_PIO2_11) &&                       \
    (LPC13xx_SPI_SCK0_SELECTOR != SCK0_IS_PIO0_6)
#error "invalid pin assigned to SCK0 signal"
#endif

/**
 * @brief   SSP0 clock.
 */
#define LPC17xx_SSP_SPI0_PCLK                                               \
  (LPC13xx_MAINCLK / LPC17xx_SSP_SPI0CLKDIV)

/**
 * @brief   SSP1 clock.
 */
#define LPC17xx_SSP_SPI1_PCLK                                               \
  (LPC13xx_MAINCLK / LPC17xx_SSP_SPI1CLKDIV)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SSPSPIDriver SSPSPIDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object triggering the
 *                      callback
 */
typedef void (*spicallback_t)(SSPSPIDriver *spip);

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
  ioportid_t            ssport;
  /**
   * @brief The chip select line pad number.
   */
  uint16_t              sspad;
  /**
   * @brief SSP CR0 initialization data.
   */
  uint16_t              cr0;
  /**
   * @brief SSP Bit Rate initialization data.
   */
  uint32_t              bit_rate;
} SSPSPIConfig;

/**
 * @brief   Structure representing a SPI driver.
 */
struct SSPSPIDriver {
  /**
   * @brief Driver state.
   */
  sspspistate_t            state;
  /**
   * @brief Current configuration data.
   */
  const SSPSPIConfig       *config;
#if SSP_SPI_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  Thread                *thread;
#endif /* SSP_SPI_USE_WAIT */
#if SSP_SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the bus.
   */
  Mutex                 mutex;
#elif CH_USE_SEMAPHORES
  Semaphore             semaphore;
#endif
#endif /* SSP_SPI_USE_MUTUAL_EXCLUSION */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the SSP registers block.
   */
  LPC_SSP_TypeDef       *ssp;
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

#if LPC17xx_USE_SSP_SPI0 && !defined(__DOXYGEN__)
extern SSPSPIDriver SPID1;
#endif

#if LPC17xx_USE_SSP_SPI1 && !defined(__DOXYGEN__)
extern SSPSPIDriver SPID2;
#endif

#ifdef LPC177x_8x
#if LPC17xx_USE_SSP_SPI2 && !defined(__DOXYGEN__)
extern SSPSPIDriver SPID3;
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void ssp_spi_lld_init(void);
  void ssp_spi_lld_start(SSPSPIDriver *spip);
  void ssp_spi_lld_stop(SSPSPIDriver *spip);
  void ssp_spi_lld_select(SSPSPIDriver *spip);
  void ssp_spi_lld_unselect(SSPSPIDriver *spip);
  void ssp_spi_lld_ignore(SSPSPIDriver *spip, size_t n);
  void ssp_spi_lld_exchange(SSPSPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void ssp_spi_lld_send(SSPSPIDriver *spip, size_t n, const void *txbuf);
  void ssp_spi_lld_receive(SSPSPIDriver *spip, size_t n, void *rxbuf);
  uint16_t ssp_spi_lld_polled_exchange(SSPSPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _ssp_spi_lld_H_ */

/** @} */
