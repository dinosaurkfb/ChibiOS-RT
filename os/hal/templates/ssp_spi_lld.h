
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
 * @file    LPC13xx/ssp_spi_lld.h
 * @brief   LPC13xx low level SSP_SPI driver header.
 *
 * @addtogroup SSP_SPI
 * @{
 */

#ifndef _ssp_spi_lld_H_
#define _ssp_spi_lld_H_

#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)



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
   * @brief SSP CPSR initialization data.
   */
  uint32_t              cpsr;
} SSPSPIConfig;

/**
 * @brief   Structure representing a SPI driver.
 */
struct SSPSPIDriver {
  /**
   * @brief Driver state.
   */
  spistate_t            state;
  /**
   * @brief Current configuration data.
   */
  const SSPSPIConfig       *config;
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

