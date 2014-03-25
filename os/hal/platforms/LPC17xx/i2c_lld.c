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
 * @file    templates/i2c_lld.c
 * @brief   I2C Driver subsystem low level driver source template.
 *
 * @addtogroup I2C
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)
#include "pinsel_lld.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define START_RETRY_NUM 10

#define EVT_START 1
#define EVT_STOP 2
#define EVT_WAIT_STOP 3
#define EVT_IIC_ISR 4
#define EVT_TIMEOUT1 5
#define EVT_TIMEOUT2 6
#define EVT_TIMEOUT3 7
#define EVT_FINISH 8
#define EVT_RET 9
#define EVT_TIME_ISR 10
#define EVT_FINISH_BEGIN 11
#define EVT_DATA  12

#define I2C_STAT_IDLE 0
#define I2C_STAT_WAIT_STOP 1
#define I2C_STAT_WAIT_START 2
#define I2C_STAT_RUN 3

#define I2C_RETRY_INTERVAL 2000

#define EnableI2CInt() NVIC_EnableIRQ(I2C0_IRQn + i2cp->offset)
#define DisableI2CInt() NVIC_DisableIRQ(I2C0_IRQn + i2cp->offset)

#define ResetI2CTimer() {if (chVTIsArmedI(&(i2cp->vt))) chVTResetI(&(i2cp->vt)); \
    chVTSetI(&(i2cp->vt), US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, (void *)i2cp);}

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C0 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C0 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/**
 * @brief   I2C1 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/**
 * @brief   I2C2 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C2 || defined(__DOXYGEN__)
I2CDriver I2CD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
static void i2c_tm_cb(void *arg);

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#ifdef DEBUG_I2C
struct I2CLog {
  int event;
  uint32_t data1;
};
typedef struct I2CLog I2CLog;
#define I2C_LOG_NUM 1000
static I2CLog s_log[I2C_LOG_NUM];
static int s_log_idx;

static void AddLog(int event, uint32_t data)
{
  if(s_log_idx < I2C_LOG_NUM) {
    s_log[s_log_idx].data1 = data;
    s_log[s_log_idx].event = event;
    s_log_idx++;
  }
}

static char *StrEvt(int evt)
{
  switch(evt) {
    case EVT_START:
      return "start    ";
    case EVT_STOP:
      return "stop     ";
    case EVT_WAIT_STOP:
      return "wait stop";
    case EVT_IIC_ISR:
      return "iic isr  ";
    case EVT_TIME_ISR:
      return "time isr ";
    case EVT_TIMEOUT1:
      return "stop timeout1 ";
    case EVT_TIMEOUT2:
      return "start timeout2 ";
    case EVT_TIMEOUT3:
      return "run timeout3 ";
    case EVT_FINISH:
      return "finish   ";
    case EVT_RET:
      return "return   ";
    case EVT_FINISH_BEGIN:
      return "enter finish";
    case EVT_DATA:
      return "DATA ";
  }
  return     "         ";
}

void I2C_Dump(void)
{
  int i = 0;
  for(; i<s_log_idx; i++) {
    I2CLog *log = &s_log[i];
    LOG_PRINT("evt=%s data=%04x\n", StrEvt(log->event), log->data1);
  }
  LOG_PRINT("\n");
}
#else
#define AddLog(a, b)
void I2C_Dump(void) {}
#endif

static void dummy(void *arg) {
  (void)arg;
}

static void StartI2C(I2CDriver *i2cp)
{
  i2cp->rwBytes = 0;
  i2cp->errors = I2CD_NO_ERROR;

  /* Enable I2C interface, and start i2c in master mode */
  i2cp->reg.stat = I2C_STAT_WAIT_START;
  i2cp->i2c->CONCLR = I2CF_EN | I2CF_STA | I2CF_SI | I2CF_AA;
  i2cp->i2c->CONSET = I2CF_EN | I2CF_STA;
  EnableI2CInt();
  AddLog(EVT_START, 0);
}

static void TryStartI2C(I2CDriver *i2cp)
{
  /* Transmit a stop bit to recover*/
  if( (i2cp->i2c->CONSET & I2CF_STO) ) {
    i2cp->reg.stat = I2C_STAT_WAIT_STOP;
    AddLog(EVT_WAIT_STOP, 0);
  }
  else
    StartI2C(i2cp);
}

static void TryReStartI2C(I2CDriver *i2cp)
{
  AddLog(EVT_STOP, 0);
  i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
  i2cp->i2c->CONSET = I2CF_STO;
  /* Transmit a stop bit to recover*/
  if( (i2cp->i2c->CONSET & I2CF_STO) ) {
    /* Recover failed */
    DisableI2CInt();
    i2cp->reg.stat = I2C_STAT_WAIT_STOP;
    AddLog(EVT_WAIT_STOP, 0);
  }
  else
    /* Recovered, start again */
    StartI2C(i2cp);
}

static void FinishI2C(I2CDriver *i2cp, int err)
{
  /* Release the binary semaphore, so the wait can return */
  chBSemSignalI(&(i2cp->done));
  AddLog(EVT_FINISH_BEGIN, 0);
  i2cp->reg.stat = 0;
  i2cp->errors = err;

  if (chVTIsArmedI(&(i2cp->vt))) {
    chVTResetI(&(i2cp->vt));
  }
  DisableI2CInt();

  /* Stop the bus */
  if(err != I2CD_NO_ERROR || i2cp->reg.bStop) {
    AddLog(EVT_STOP, 0);
    i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI | I2CF_AA;
    i2cp->i2c->CONSET = I2CF_STO;
  }

  AddLog(EVT_FINISH, 0);
}

/*
 * I2C timer callback.
 */
static void i2c_tm_cb(void *arg) {
  I2CDriver *i2cp =(I2CDriver *)arg;
  chSysLockFromIsr();
  ResetI2CTimer();
  AddLog(EVT_TIME_ISR, 0);

  switch(i2cp->reg.stat) {
    case I2C_STAT_WAIT_STOP:
      AddLog(EVT_TIMEOUT1, 0);
      if( i2cp->i2c->CONSET & I2CF_STO ) {
        FinishI2C(i2cp, E_I2C_TIMEOUT2);
      } else {
        StartI2C(i2cp);
      }
      break;
    case I2C_STAT_WAIT_START:
      AddLog(EVT_TIMEOUT2, 0);
      if(i2cp->reg.retry_start > 0) {
        i2cp->reg.retry_start--;
        TryReStartI2C(i2cp);
      }
      else {
        FinishI2C(i2cp, E_I2C_TIMEOUT);
      }
      break;
    case I2C_STAT_RUN:
      AddLog(EVT_TIMEOUT3, 0);
      if(i2cp->reg.retry_start > 0) {
        i2cp->reg.retry_start--;
        TryReStartI2C(i2cp);
      }
      else {
        FinishI2C(i2cp, E_I2C_TIMEOUT1);
      }
      break;
  }

  chSysUnlockFromIsr();
}

/* This function is for internal use. */
int Locked_I2C_Request(I2CDriver *i2cp, uint8_t devAddr, uint8_t *buf, int len, int bRead, int bStop, int retry, int needAck)
{
  chSysLock();
#ifdef DEBUG_I2C
  s_log_idx=0;
#endif

  i2cp->reg.devAddr = devAddr;
  if (bRead == I2C_B_READ) {
    i2cp->reg.rxbuf = buf;
  } else {
    i2cp->reg.txbuf = buf;
  }
  i2cp->reg.len = len;
  i2cp->reg.bRead = bRead;
  i2cp->reg.bStop = bStop;
  i2cp->reg.retry_start = START_RETRY_NUM;
  i2cp->reg.retry_run = retry;
  i2cp->reg.needAck = needAck;
  chBSemResetI(&(i2cp->done), TRUE);

  if (chVTIsArmedI(&(i2cp->vt))) {
    chVTResetI(&(i2cp->vt));
  }
  chVTSetI(&(i2cp->vt), US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, (void *)i2cp);

  TryStartI2C(i2cp);
  chSysUnlock();

  int ret = chBSemWaitTimeoutS(&(i2cp->done), MS2ST(100));
  AddLog(EVT_RET, ret);

  ret = (i2cp->errors != I2CD_NO_ERROR) ? i2cp->errors : I2CD_NO_ERROR;
  AddLog(EVT_RET, ret);
  return ret;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an I2C I/O block
 * @param[in] sdp       communication channel associated to the I2C
 */
static void serve_interrupt(I2CDriver *i2cp) {
  chSysLockFromIsr();
  int32_t err = 0;
  ResetI2CTimer();

  uint8_t stat = i2cp->i2c->STAT;
  AddLog(EVT_IIC_ISR, stat);

  switch(stat) {
    case 0x00:
      err = I2CD_BUS_ERROR;
      break;

    case 0x08: /* START sent */
    case 0x10: /* ReSTART sent */
      i2cp->reg.stat = I2C_STAT_RUN;
      AddLog(EVT_DATA, (i2cp->reg.devAddr<<1) | i2cp->reg.bRead);
      /* send devAddr + R/W */
      i2cp->i2c->DAT = (i2cp->reg.devAddr<<1) | i2cp->reg.bRead;
      i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
      break;

    case 0x20: /* addr+W sent, NOT ACK has been received */
    case 0x30: /* data sent£¬NOT ACK has been received */
      if(i2cp->reg.needAck) {
        err = I2CD_ACK_FAILURE;
        break;
      }

    case 0x18: /* addr+W sent, ACK has been received */
    case 0x28: /* data sent£¬ACK has been received */
      if(i2cp->rwBytes < i2cp->reg.len) {
        /* AddLog(EVT_DATA, i2cp->reg.txbuf[i2cp->rwBytes]); */
        i2cp->i2c->DAT = i2cp->reg.txbuf[i2cp->rwBytes++];
        i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
      }
      else {
        /* Transmit finish */
        FinishI2C(i2cp, I2CD_NO_ERROR);
      }
      break;

    case 0x38: /*arbitration lost in SLA+R/W or data bytes */
      err = I2CD_ARBITRATION_LOST;
      break;

    case 0x48: /* addr+R transmitted, NOT ACK has been received */
      if(i2cp->reg.needAck) {
        err = I2CD_ACK_FAILURE;
        break;
      }

    case 0x40: /* addr+R transmitted, ACK has been received */
      if(i2cp->reg.len > 1) {
        /* Acknowledge the next received byte */
        i2cp->i2c->CONSET = I2CF_AA;
      }
      i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
      break;

    case 0x50: /* Data received, ACK has been returned */
      /* AddLog(EVT_DATA, i2cp->rwBytes); */
      i2cp->reg.rxbuf[i2cp->rwBytes++] = (uint8_t)(i2cp->i2c->DAT);
      i2cp->i2c->CONCLR = (i2cp->rwBytes == i2cp->reg.len-1) ? (I2CF_STA|I2CF_SI|I2CF_AA) : (I2CF_STA|I2CF_SI);
      break;

    case 0x58: /* Data received, NOT ACK has been returned */
      i2cp->reg.rxbuf[i2cp->rwBytes++] = (uint8_t)(i2cp->i2c->DAT);
      FinishI2C(i2cp, I2CD_NO_ERROR);
      break;

    default:
      err = E_I2C_STAT;
  }

  if (err != I2CD_NO_ERROR) {
    if (i2cp->reg.retry_run>0) {
      AddLog(EVT_DATA, i2cp->reg.retry_run);
      i2cp->reg.retry_run--;
      i2cp->reg.retry_start = START_RETRY_NUM;
      TryReStartI2C(i2cp);
    } else {
      FinishI2C(i2cp, err);
    }
  }
  chSysUnlockFromIsr();
}

/**
 * @brief   I2C0 IRQ handler.
 *
 * @isr
 */
#if LPC17xx_I2C_USE_I2C0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(Vector68) {

  CH_IRQ_PROLOGUE();
  serve_interrupt(&I2CD1);
  CH_IRQ_EPILOGUE();
}
#endif

/**
 * @brief   I2C1 IRQ handler.
 *
 * @isr
 */
#if LPC17xx_I2C_USE_I2C1 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(Vector6C) {

  CH_IRQ_PROLOGUE();
  serve_interrupt(&I2CD2);
  CH_IRQ_EPILOGUE();
}
#endif

/**
 * @brief   I2C2 IRQ handler.
 *
 * @isr
 */
#if LPC17xx_I2C_USE_I2C2 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(Vector70) {

  CH_IRQ_PROLOGUE();
  serve_interrupt(&I2CD3);
  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {
#if LPC17xx_I2C_USE_I2C0
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = (LPC_I2C_TypeDef*) LPC_I2C0;
  I2CD1.offset = i2c0;
#endif /* LPC17xx_I2C_USE_I2C0 */

#if LPC17xx_I2C_USE_I2C1
  i2cObjectInit(&I2CD2);
  I2CD2.i2c = (LPC_I2C_TypeDef*) LPC_I2C1;
  I2CD2.offset = i2c1;
#endif /* LPC17xx_I2C_USE_I2C1 */

#if LPC17xx_I2C_USE_I2C2
  i2cObjectInit(&I2CD3);
  I2CD3.i2c = (LPC_I2C_TypeDef*) LPC_I2C2;
  I2CD3.offset = i2c2;
#endif /* LPC17xx_I2C_USE_I2C2 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {
  chBSemInit(&(i2cp->done), FALSE);
  uint32_t i2cscl;
  uint32_t mulh, mull, div;

  if (i2cp->state == I2C_STOP) {
    /* Enables the peripheral.*/
#if LPC17xx_I2C_USE_I2C0
    if (&I2CD1 == i2cp) {
      LPC_SC->PCONP |= (1 << 7);
      /* set PIO0.27 and PIO0.28 to I2C0 SDA and SCL */
#ifdef LPC177x_8x
      PINSEL_ConfigPin(0, 27, 0b001);
      PINSEL_ConfigPin(0, 28, 0b001);
#else /* ifdef LPC177x_8x */
      /* function to 01 on both SDA and SCL. */
      LPC_PINCON->PINSEL1 &= ~0x03C00000;
      LPC_PINCON->PINSEL1 |= 0x01400000;
#endif /* ifdef LPC177x_8x */
    }
#endif /* LPC17xx_I2C_USE_I2C0 */

#if LPC17xx_I2C_USE_I2C1
    if (&I2CD2 == i2cp) {
      LPC_SC->PCONP |=(1<<19);
#ifdef LPC177x_8x
      /* set PIO2.14 and PIO2.15 to I2C1 SDA and SCL */
      PINSEL_ConfigPin(2, 14, 0b010);
      PINSEL_ConfigPin(2, 15, 0b010);
#else /* ifdef LPC177x_8x */
      /* set P0.19 and P0.20 to I2C2 SDA and SCL */
      LPC_PINCON->PINSEL1 |=((0x03<<6)|(0x03<<8));
#endif /* ifdef LPC177x_8x */
    }
#endif /* LPC17xx_I2C_USE_I2C1 */

#if LPC17xx_I2C_USE_I2C2
    if (&I2CD3 == i2cp) {
      LPC_SC->PCONP |= (1 << 26);
#ifdef LPC177x_8x
      /* set PIO2.30 and PIO2.31 to I2C2 SDA and SCL */
      PINSEL_ConfigPin(2, 30, 0b010);
      PINSEL_ConfigPin(2, 31, 0b010);
#else /* ifdef LPC177x_8x */
      /* set PIO0.10 and PIO0.11 to I2C2 SDA and SCL */
      LPC_PINCON->PINSEL1 &= ~0x00F00000;
      LPC_PINCON->PINSEL1 |= 0x00A00000;
#endif /* ifdef LPC177x_8x */
    }
#endif /* LPC17xx_I2C_USE_I2C2 */

    /*--- Clear flags ---*/
    i2cp->i2c->CONCLR = I2CF_AA | I2CF_SI | I2CF_STA | I2CF_EN;

    /* Setup I2C clock parameters.*/
    i2cscl = (LPC17xx_PCLK/(i2cp->config->clock_timing));
    if (i2cp->config->mode == I2C_FAST_MODE) {
      div = 19;
      mull = 13;
      mulh = 6;
    } else if (i2cp->config->mode == I2C_FAST_MODE_PLUS) {
      div = 3;
      mull = 2;
      mulh = 1;
    } else { /* i2cp->config->mode == I2C_STANDARD_MODE */
      div = 2;
      mull = 1;
      mulh = 1;
    }

    i2cp->i2c->SCLH = (mulh * i2cscl) / div;
    i2cp->i2c->SCLL = (mull * i2cscl) / div;

#if LPC17xx_I2C_USE_I2C0
    nvicEnableVector(I2C0_IRQn,
                     CORTEX_PRIORITY_MASK(LPC17xx_I2C0_IRQ_PRIORITY));
#endif
#if LPC17xx_I2C_USE_I2C1
    nvicEnableVector(I2C1_IRQn,
                     CORTEX_PRIORITY_MASK(LPC17xx_I2C1_IRQ_PRIORITY));
#endif
#if LPC17xx_I2C_USE_I2C2
    nvicEnableVector(I2C2_IRQn,
                     CORTEX_PRIORITY_MASK(LPC17xx_I2C2_IRQ_PRIORITY));
#endif
  }
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if LPC17xx_I2C_USE_I2C0
    if (&I2CD1 == i2cp) {
      LPC_SC->PCONP &= ~(1 << 7);
      nvicDisableVector(I2C0_IRQn);
    }
#endif /* LPC17xx_I2C_USE_I2C0 */

#if LPC17xx_I2C_USE_I2C1
    if (&I2CD2 == i2cp) {
      LPC_SC->PCONP &= ~(1 << 19);
      nvicDisableVector(I2C1_IRQn);
    }
#endif /* LPC17xx_I2C_USE_I2C1 */

#if LPC17xx_I2C_USE_I2C2
    if (&I2CD3 == i2cp) {
      LPC_SC->PCONP &= ~(1 << 26);
      nvicDisableVector(I2C2_IRQn);
    }
#endif /* LPC17xx_I2C_USE_I2C2 */
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 * @details
 *
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     systime_t timeout) {

  static VirtualTimer vt;
  chVTSetI(&vt, timeout, dummy, NULL);

  /* These settings won't be changed until return. */
  i2cp->reg.devAddr = addr;
  i2cp->reg.rxbuf = rxbuf;
  i2cp->reg.len = rxbytes;
  i2cp->reg.bRead = I2C_B_READ;
  i2cp->reg.bStop = I2C_B_STOP1;
  i2cp->reg.needAck = I2C_B_NEEDACK;

  while (1) {
    /* Reset all retry settings */
    i2cp->reg.retry_start = START_RETRY_NUM;
    i2cp->reg.retry_run = 2;

#ifdef DEBUG_I2C
    s_log_idx=0;
#endif
    chBSemResetI(&(i2cp->done), TRUE);

    if (chVTIsArmedI(&(i2cp->vt))) {
      chVTResetI(&(i2cp->vt));
    }
    chVTSetI(&(i2cp->vt), US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, (void *)i2cp);

    TryStartI2C(i2cp);

    /* Wait I2C finish */
    chBSemWaitS(&(i2cp->done));
    if (i2cp->errors != I2CD_NO_ERROR) {
      if (!chVTIsArmedI(&vt)) {
        AddLog(EVT_RET, i2cp->errors);
        return RDY_TIMEOUT;
      }
    } else {
      AddLog(EVT_RET, RDY_OK);
      return RDY_OK;
    }
  }
}

/**
 * @brief   Transmits data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on STM32F1x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval RDY_OK       if the function succeeded.
 * @retval RDY_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval RDY_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {

  static VirtualTimer vt;
  chVTSetI(&vt, timeout, dummy, NULL);

  /* These settings won't be changed until return. */
  i2cp->reg.devAddr = addr;
  i2cp->reg.txbuf = txbuf;
  i2cp->reg.len = txbytes;
  i2cp->reg.bRead = I2C_B_WRITE;
  i2cp->reg.bStop = I2C_B_STOP1;
  i2cp->reg.needAck = I2C_B_NEEDACK;
  chBSemResetI(&(i2cp->done), TRUE);
  volatile uint32_t i = 0;

  while (1) {
    /* Reset all retry settings */
    i2cp->reg.retry_start = START_RETRY_NUM;
    i2cp->reg.retry_run = 2;

#ifdef DEBUG_I2C
    s_log_idx=0;
#endif

    if (chVTIsArmedI(&(i2cp->vt))) {
      chVTResetI(&(i2cp->vt));
    }
    chVTSetI(&(i2cp->vt), US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, (void *)i2cp);

    TryStartI2C(i2cp);
    chSysUnlock();
    /* Wait I2C finish */
    chBSemWait(&(i2cp->done));
    for (i = 0; i < 5000; i++);
    chSysLock();
    if (i2cp->errors != I2CD_NO_ERROR) {
      if (!chVTIsArmedI(&vt)) {
        AddLog(EVT_RET, i2cp->errors);
        return RDY_TIMEOUT;
      }
    } else {
      /* Continue to receive from I2C bus */
      if (rxbytes != 0) {
        i2cp->reg.rxbuf = rxbuf;
        i2cp->reg.len = rxbytes;
        i2cp->reg.bRead = I2C_B_READ;
        /* Receiving process has been started. */
        rxbytes = 0;
        continue;
      }
      AddLog(EVT_RET, RDY_OK);
      return RDY_OK;
    }
  }
}

#endif /* HAL_USE_I2C */

/** @} */
