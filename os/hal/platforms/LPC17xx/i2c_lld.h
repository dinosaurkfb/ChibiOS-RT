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
 * @file    templates/i2c_lld.h
 * @brief   I2C Driver subsystem low level driver header template.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef _I2C_LLD_H_
#define _I2C_LLD_H_

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define DEBUG_I2C  TRUE

#define I2CF_AA                 (1<<2)
#define I2CF_SI                 (1<<3)
#define I2CF_STO                (1<<4)
#define I2CF_STA                (1<<5)
#define I2CF_EN                 (1<<6)
#define I2CF_CLR_ALL            0x2C

#define I2DAT_I2C               0x00000000  /* I2C Data Reg */
#define I2ADR_I2C               0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH             (LPC17xx_PCLK/200000) //I2C clock 100 khz
#define I2SCLL_SCLL             (LPC17xx_PCLK/200000) //I2C clock 100 khz


//I2C
#define E_I2C_BUS            1000
#define E_I2C_NACK           1001
#define E_I2C_ARB            1002
#define E_I2C_STAT           1003
#define E_I2C_TIMEOUT        1004
#define E_I2C_TIMEOUT1       1005
#define E_I2C_TIMEOUT2       1006

#define I2C_B_READ     1
#define I2C_B_WRITE    0
#define I2C_B_STOP1    1
#define I2C_B_STOP0    0
#define I2C_B_NEEDACK  1
#define I2C_B_NOACK    0
/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   I2C0 driver enable switch.
 * @details If set to @p TRUE the support for I2C0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LPC17xx_I2C_USE_I2C0) || defined(__DOXYGEN__)
#define LPC17xx_I2C_USE_I2C0               FALSE
#endif
/** @} */

/**
 * @brief   I2C1 driver enable switch.
 * @details If set to @p TRUE the support for I2C1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LPC17xx_I2C_USE_I2C1) || defined(__DOXYGEN__)
#define LPC17xx_I2C_USE_I2C1               FALSE
#endif
/** @} */

/**
 * @brief   I2C2 driver enable switch.
 * @details If set to @p TRUE the support for I2C2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LPC17xx_I2C_USE_I2C2) || defined(__DOXYGEN__)
#define LPC17xx_I2C_USE_I2C2               FALSE
#endif
/** @} */

/**
 * @brief   I2C0 interrupt priority level setting.
 */
#if !defined(LPC17xx_I2C0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_I2C0_IRQ_PRIORITY   3
#endif

/**
 * @brief   I2C1 interrupt priority level setting.
 */
#if !defined(LPC17xx_I2C1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_I2C1_IRQ_PRIORITY   3
#endif

/**
 * @brief   I2C2 interrupt priority level setting.
 */
#if !defined(LPC17xx_I2C2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC17xx_I2C2_IRQ_PRIORITY   3
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing I2C address.
 */
typedef uint16_t i2caddr_t;

/**
 * @brief   Type of I2C Driver condition flags.
 */
typedef uint32_t i2cflags_t;

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */

/**
 * @brief Driver configuration structure.
 */
typedef struct {
} I2CConfig;


/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct I2CDriver I2CDriver;

/**
 * @brief Structure representing an I2C driver.
 */
struct I2CDriver {
  /**
   * @brief   Driver state.
   *          0=idle 
   *          1=wait for i2c stop
   *          2=wait for i2c start ok
   *          3=normal
   */
  i2cstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2cflags_t                errors;
#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  Mutex                     mutex;
#elif CH_USE_SEMAPHORES
  Semaphore                 semaphore;
#endif
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/

  /* Pointer to the I2C registers block.*/                                \
  LPC_I2C_TypeDef        *i2c;
};

struct i2c_reg
{
	int stat;//0=idle 1=wait for i2c stop,2=wait for i2c start ok 3=normal
	int len;
	int bRead;
	int bStop;
	int rwBytes;
	int err;
	int retry_start;
	int retry_run;
	int needAck;
	uint8_t devAddr;
	uint8_t *buf;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
#if LPC17xx_I2C_USE_I2C0
extern I2CDriver I2CD0;
#endif
#if LPC17xx_I2C_USE_I2C1
extern I2CDriver I2CD1;
#endif
#if LPC17xx_I2C_USE_I2C2
extern I2CDriver I2CD2;
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        systime_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       systime_t timeout);

  int Locked_I2C_Request(uint8_t devAddr, uint8_t *buf, int len, int bRead, int bStop, int retry, int needAck);
#ifdef DEBUG_I2C
  void I2C_Dump(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C */

#endif /* _I2C_LLD_H_ */

/** @} */
