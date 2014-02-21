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
 * @file    spi.h
 * @brief   SSP_SPI Driver macros and structures.
 *
 * @addtogroup SSP_SPI
 * @{
 */

#ifndef _SSP_SPI_H_
#define _SSP_SPI_H_

#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SSP_SPI configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SSP_SPI_USE_WAIT) || defined(__DOXYGEN__)
#define SSP_SPI_USE_WAIT                TRUE
#endif

/**
 * @brief   Enables the @p spiAcquireBus() and @p spiReleaseBus() APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SSP_SPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define SSP_SPI_USE_MUTUAL_EXCLUSION    TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SSP_SPI_USE_MUTUAL_EXCLUSION && !CH_USE_MUTEXES && !CH_USE_SEMAPHORES
#error "SSP_SPI_USE_MUTUAL_EXCLUSION requires CH_USE_MUTEXES and/or CH_USE_SEMAPHORES"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  SSP_SPI_UNINIT = 0,                   /**< Not initialized.                   */
  SSP_SPI_STOP = 1,                     /**< Stopped.                           */
  SSP_SPI_READY = 2,                    /**< Ready.                             */
  SSP_SPI_ACTIVE = 3,                   /**< Exchanging data.                   */
  SSP_SPI_COMPLETE = 4                  /**< Asynchronous operation complete.   */
} spistate_t;

#include "spi_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @iclass
 */
#define spiSelectI(spip) {                                                  \
  spi_lld_select(spip);                                                     \
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @iclass
 */
#define spiUnselectI(spip) {                                                \
  spi_lld_unselect(spip);                                                   \
}

/**
 * @brief   Ignores data on the SSP_SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SSP_SPI bus and ignores the received data.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @iclass
 */
#define spiStartIgnoreI(spip, n) {                                          \
  (spip)->state = SSP_SPI_ACTIVE;                                               \
  spi_lld_ignore(spip, n);                                                  \
}

/**
 * @brief   Exchanges data on the SSP_SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
#define spiStartExchangeI(spip, n, txbuf, rxbuf) {                          \
  (spip)->state = SSP_SPI_ACTIVE;                                               \
  spi_lld_exchange(spip, n, txbuf, rxbuf);                                  \
}

/**
 * @brief   Sends data over the SSP_SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @iclass
 */
#define spiStartSendI(spip, n, txbuf) {                                     \
  (spip)->state = SSP_SPI_ACTIVE;                                               \
  spi_lld_send(spip, n, txbuf);                                             \
}

/**
 * @brief   Receives data from the SSP_SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p spiSelect() or
 *          @p spiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
#define spiStartReceiveI(spip, n, rxbuf) {                                  \
  (spip)->state = SSP_SPI_ACTIVE;                                               \
  spi_lld_receive(spip, n, rxbuf);                                          \
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 * @note    This API is implemented as a macro in order to minimize latency.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 * @param[in] frame     the data frame to send over the SSP_SPI bus
 * @return              The received data frame from the SSP_SPI bus.
 */
#define spiPolledExchange(spip, frame) spi_lld_polled_exchange(spip, frame)
/** @} */

/**
 * @name    Low Level driver helper macros
 * @{
 */
#if SSP_SPI_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Waits for operation completion.
 * @details This function waits for the driver to complete the current
 *          operation.
 * @pre     An operation must be running while the function is invoked.
 * @note    No more than one thread can wait on a SSP_SPI driver using
 *          this function.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
#define _spi_wait_s(spip) {                                                 \
  chDbgAssert((spip)->thread == NULL,                                       \
              "_spi_wait(), #1", "already waiting");                        \
  (spip)->thread = chThdSelf();                                             \
  chSchGoSleepS(THD_STATE_SUSPENDED);                                       \
}

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
#define _spi_wakeup_isr(spip) {                                             \
  chSysLockFromIsr();                                                       \
  if ((spip)->thread != NULL) {                                             \
    Thread *tp = (spip)->thread;                                            \
    (spip)->thread = NULL;                                                  \
    tp->p_u.rdymsg = RDY_OK;                                                \
    chSchReadyI(tp);                                                        \
  }                                                                         \
  chSysUnlockFromIsr();                                                     \
}
#else /* !SSP_SPI_USE_WAIT */
#define _spi_wait_s(spip)
#define _spi_wakeup_isr(spip)
#endif /* !SSP_SPI_USE_WAIT */

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] spip      pointer to the @p SSPSPIDriver object
 *
 * @notapi
 */
#define _spi_isr_code(spip) {                                               \
  if ((spip)->config->end_cb) {                                             \
    (spip)->state = SSP_SPI_COMPLETE;                                           \
    (spip)->config->end_cb(spip);                                           \
    if ((spip)->state == SSP_SPI_COMPLETE)                                      \
      (spip)->state = SSP_SPI_READY;                                            \
  }                                                                         \
  else                                                                      \
    (spip)->state = SSP_SPI_READY;                                              \
  _spi_wakeup_isr(spip);                                                    \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sspspiInit(void);
  void sspspiObjectInit(SSPSPIDriver *spip);
  void sspspiStart(SSPSPIDriver *spip, const SPIConfig *config);
  void sspspiStop(SSPSPIDriver *spip);
  void sspspiSelect(SSPSPIDriver *spip);
  void sspspiUnselect(SSPSPIDriver *spip);
  void sspspiStartIgnore(SSPSPIDriver *spip, size_t n);
  void sspspiStartExchange(SSPSPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void sspspiStartSend(SSPSPIDriver *spip, size_t n, const void *txbuf);
  void sspspiStartReceive(SSPSPIDriver *spip, size_t n, void *rxbuf);
#if SPI_USE_WAIT
  void sspspiIgnore(SSPSPIDriver *spip, size_t n);
  void sspspiExchange(SSPSPIDriver *spip, size_t n, const void *txbuf, void *rxbuf);
  void sspspiSend(SSPSPIDriver *spip, size_t n, const void *txbuf);
  void sspspiReceive(SSPSPIDriver *spip, size_t n, void *rxbuf);
#endif /* SPI_USE_WAIT */
#if SPI_USE_MUTUAL_EXCLUSION
  void sspspiAcquireBus(SSPSPIDriver *spip);
  void sspspiReleaseBus(SSPSPIDriver *spip);
#endif /* SPI_USE_MUTUAL_EXCLUSION */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI_H_ */

/** @} */
