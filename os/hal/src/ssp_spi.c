/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/**
 * @file    ssp_spi.c
 * @brief   SSP SSP_SPI Driver code.
 *
 * @addtogroup SSP_SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   SPI Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void sspspiInit(void) {

  ssp_spi_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SSPSPIDriver structure.
 *
 * @param[out] spip     pointer to the @p SSPSPIDriver object
 *
 * @init
 */
void sspspiObjectInit(SSPSPIDriver *spip) {

  spip->state = SSP_SPI_STOP;
  spip->config = NULL;
#if SSP_SPI_USE_WAIT
  spip->thread = NULL;
#endif /* SSP_SPI_USE_WAIT */
#if SSP_SPI_USE_MUTUAL_EXCLUSION
#if CH_USE_MUTEXES
  chMtxInit(&spip->mutex);
#else
  chSemInit(&spip->semaphore, 1);
#endif
#endif /* SSP_SPI_USE_MUTUAL_EXCLUSION */
#if defined(SSP_SPI_DRIVER_EXT_INIT_HOOK)
  SSP_SPI_DRIVER_EXT_INIT_HOOK(spip);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] config    pointer to the @p SPIConfig object
 *
 * @api
 */
void sspspiStart(SSPSPIDriver *spip, const SSPSPIConfig *config) {

  chDbgCheck((spip != NULL) && (config != NULL), "spiStart");

  chSysLock();
  chDbgAssert((spip->state == SSP_SPI_STOP) || (spip->state == SSP_SPI_READY),
              "spiStart(), #1", "invalid state");
  spip->config = config;
  ssp_spi_lld_start(spip);
  spip->state = SSP_SPI_READY;
  chSysUnlock();
}

/**
 * @brief Deactivates the SPI peripheral.
 * @note  Deactivating the peripheral also enforces a release of the slave
 *        select line.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @api
 */
void sspspiStop(SSPSPIDriver *spip) {

  chDbgCheck(spip != NULL, "spiStop");

  chSysLock();
  chDbgAssert((spip->state == SSP_SPI_STOP) || (spip->state == SSP_SPI_READY),
              "spiStop(), #1", "invalid state");
  ssp_spi_lld_unselect(spip);
  ssp_spi_lld_stop(spip);
  spip->state = SSP_SPI_STOP;
  chSysUnlock();
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @api
 */
void sspspiSelect(SSPSPIDriver *spip) {

  chDbgCheck(spip != NULL, "sspspiSelect");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "sspspiSelect(), #1", "not ready");
  sspspiSelectI(spip);
  chSysUnlock();
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @api
 */
void sspspiUnselect(SSPSPIDriver *spip) {

  chDbgCheck(spip != NULL, "spiUnselect");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiUnselect(), #1", "not ready");
  sspspiUnselectI(spip);
  chSysUnlock();
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p sspspiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @api
 */
void sspspiStartIgnore(SSPSPIDriver *spip, size_t n) {

  chDbgCheck((spip != NULL) && (n > 0), "spiStartIgnore");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiStartIgnore(), #1", "not ready");
  sspspiStartIgnoreI(spip, n);
  chSysUnlock();
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @pre     A slave must have been selected using @p sspspiSelect() or
 *          @p sspspiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void sspspiStartExchange(SSPSPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (rxbuf != NULL) && (txbuf != NULL),
             "spiStartExchange");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiStartExchange(), #1", "not ready");
  sspspiStartExchangeI(spip, n, txbuf, rxbuf);
  chSysUnlock();
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @pre     A slave must have been selected using @p sspspiSelect() or
 *          @p sspspiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
void sspspiStartSend(SSPSPIDriver *spip, size_t n, const void *txbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (txbuf != NULL),
             "spiStartSend");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiStartSend(), #1", "not ready");
  sspspiStartSendI(spip, n, txbuf);
  chSysUnlock();
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p sspspiSelect() or
 *          @p sspspiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void sspspiStartReceive(SSPSPIDriver *spip, size_t n, void *rxbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (rxbuf != NULL),
             "spiStartReceive");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiStartReceive(), #1", "not ready");
  sspspiStartReceiveI(spip, n, rxbuf);
  chSysUnlock();
}

#if SSP_SPI_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SSP_SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @api
 */
void sspspiIgnore(SSPSPIDriver *spip, size_t n) {

  chDbgCheck((spip != NULL) && (n > 0), "spiIgnoreWait");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiIgnore(), #1", "not ready");
  chDbgAssert(spip->config->end_cb == NULL, "spiIgnore(), #2", "has callback");
  sspspiStartIgnoreI(spip, n);
  _spi_wait_s(spip);
  chSysUnlock();
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This synchronous function performs a simultaneous transmit/receive
 *          operation.
 * @pre     In order to use this function the option @p SSP_SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void sspspiExchange(SSPSPIDriver *spip, size_t n,
                 const void *txbuf, void *rxbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (rxbuf != NULL) && (txbuf != NULL),
             "spiExchange");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiExchange(), #1", "not ready");
  chDbgAssert(spip->config->end_cb == NULL,
              "spiExchange(), #2", "has callback");
  sspspiStartExchangeI(spip, n, txbuf, rxbuf);
  _spi_wait_s(spip);
  chSysUnlock();
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This synchronous function performs a transmit operation.
 * @pre     In order to use this function the option @p SSP_SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @api
 */
void sspspiSend(SSPSPIDriver *spip, size_t n, const void *txbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (txbuf != NULL), "spiSend");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiSend(), #1", "not ready");
  chDbgAssert(spip->config->end_cb == NULL, "spiSend(), #2", "has callback");
  sspspiStartSendI(spip, n, txbuf);
  _spi_wait_s(spip);
  chSysUnlock();
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This synchronous function performs a receive operation.
 * @pre     In order to use this function the option @p SSP_SPI_USE_WAIT must be
 *          enabled.
 * @pre     In order to use this function the driver must have been configured
 *          without callbacks (@p end_cb = @p NULL).
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @api
 */
void sspspiReceive(SSPSPIDriver *spip, size_t n, void *rxbuf) {

  chDbgCheck((spip != NULL) && (n > 0) && (rxbuf != NULL),
             "spiReceive");

  chSysLock();
  chDbgAssert(spip->state == SSP_SPI_READY, "spiReceive(), #1", "not ready");
  chDbgAssert(spip->config->end_cb == NULL,
              "spiReceive(), #2", "has callback");
  sspspiStartReceiveI(spip, n, rxbuf);
  _spi_wait_s(spip);
  chSysUnlock();
}
#endif /* SSP_SPI_USE_WAIT */

#if SSP_SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the SPI bus.
 * @details This function tries to gain ownership to the SPI bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p SSP_SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @api
 */
void sspspiAcquireBus(SSPSPIDriver *spip) {

  chDbgCheck(spip != NULL, "spiAcquireBus");

#if CH_USE_MUTEXES
  chMtxLock(&spip->mutex);
#elif CH_USE_SEMAPHORES
  chSemWait(&spip->semaphore);
#endif
}

/**
 * @brief   Releases exclusive access to the SPI bus.
 * @pre     In order to use this function the option @p SSP_SPI_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @api
 */
void sspspiReleaseBus(SSPSPIDriver *spip) {

  chDbgCheck(spip != NULL, "spiReleaseBus");

#if CH_USE_MUTEXES
  (void)spip;
  chMtxUnlock();
#elif CH_USE_SEMAPHORES
  chSemSignal(&spip->semaphore);
#endif
}
#endif /* SSP_SPI_USE_MUTUAL_EXCLUSION */

#endif /* HAL_USE_SSP_SPI */

/** @} */
