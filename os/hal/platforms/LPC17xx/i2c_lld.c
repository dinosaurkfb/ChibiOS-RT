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

//重试间隔=200us
#define I2C_RETRY_INTERVAL 300

#if LPC17xx_I2C_USE_I2C0
#define EnableI2CInt() nvicEnableVector(I2C0_IRQn,			\
					CORTEX_PRIORITY_MASK(LPC17xx_I2C0_IRQ_PRIORITY))
#define DisableI2CInt() nvicDisableVector(I2C0_IRQn)
#endif

#if LPC17xx_I2C_USE_I2C1
#define EnableI2CInt() NVIC_EnableIRQ(I2C1_IRQn)
/* nvicEnableVector(I2C1_IRQn,						\ */
/* 					CORTEX_PRIORITY_MASK(LPC17xx_I2C1_IRQ_PRIORITY)) */
#define DisableI2CInt() NVIC_DisableIRQ(I2C1_IRQn)
//#define DisableI2CInt() nvicDisableVector(I2C1_IRQn)
#endif

#if LPC17xx_I2C_USE_I2C2
#define EnableI2CInt() nvicEnableVector(I2C2_IRQn,			\
					CORTEX_PRIORITY_MASK(LPC17xx_I2C2_IRQ_PRIORITY))
#define DisableI2CInt() nvicDisableVector(I2C2_IRQn)
#endif

//200 * Fpclk/1000000 => 2 * Fpclk/10000
#define ResetI2CTimer() {chVTResetI(&s_vt);	\
    chVTSetI(&s_vt, US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, NULL);}

//#define EnableI2CTimer() gptStopTimer(gptp)          //允许I2C超时定时器
//#define DisableI2CTimer() gptStopTimer(s_gptp)       //禁止I2C超时定时器

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C0 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C0 || defined(__DOXYGEN__)
I2CDriver I2CD0;
#endif

/**
 * @brief   I2C1 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/**
 * @brief   I2C2 driver identifier.
 */
#if LPC17xx_I2C_USE_I2C2 || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
static volatile struct i2c_reg s_reg;
static void i2c_tm_cb(void *arg);

static VirtualTimer s_vt;
static I2CDriver *s_i2cp = NULL; 

#ifdef _TOS_
static sema_t s_lock;
#endif
static BinarySemaphore s_done;

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
    return "timeout1 ";
  case EVT_TIMEOUT2:
    return "timeout2 ";
  case EVT_TIMEOUT3:
    return "timeout3 ";
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

static void StartI2C(void)
{
  s_reg.rwBytes = 0;//读或写的字节数
  s_reg.err = 0;

  // start i2c
  s_reg.stat = I2C_STAT_WAIT_START;
  s_i2cp->i2c->CONCLR = I2CF_EN | I2CF_STA | I2CF_SI | I2CF_AA;       
  s_i2cp->i2c->CONSET = I2CF_EN | I2CF_STA; //bit6=1:使能I2C功能   bit5=1:主发送模式
  EnableI2CInt();  // 允许I2c中断
  AddLog(EVT_START, 0);
}

static void TryStartI2C(void)
{
  if( (s_i2cp->i2c->CONSET & I2CF_STO) ) {//（主）使I2C接口发送停止条件（从）从错误状态中恢复
    s_reg.stat = I2C_STAT_WAIT_STOP;
    AddLog(EVT_WAIT_STOP, 0);
  }
  else
    StartI2C();
}

static void TryReStartI2C(void)
{
  AddLog(EVT_STOP, 0);
  s_i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
  s_i2cp->i2c->CONSET = I2CF_STO;
  /* Transmit a stop bit to recover*/
  if( (s_i2cp->i2c->CONSET & I2CF_STO) ) { 
    /* Recover failed */
    DisableI2CInt(); // 禁止I2c中断
    s_reg.stat = I2C_STAT_WAIT_STOP;
    AddLog(EVT_WAIT_STOP, 0);
  }
  else
    /* Recovered, start again */
    StartI2C();
}

 static void FinishI2C(int err)
{
  //  s_done = 1;
  chBSemSignalI(&s_done);
  AddLog(EVT_FINISH_BEGIN, 0);
  s_reg.stat = 0;
  s_reg.err = err;

  //DisableI2CTimer();  // 禁止I2C超时定时器
  chVTResetI(&s_vt);
  DisableI2CInt(); // 禁止I2c中断

  if(err<0 || s_reg.bStop) {
    AddLog(EVT_STOP, 0);
    s_i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI | I2CF_AA;
    s_i2cp->i2c->CONSET = I2CF_STO;
  }

  AddLog(EVT_FINISH, 0);
}

/*
 * I2C timer callback.
 */
static void i2c_tm_cb(void *arg) {
  (void)arg;
  chSysLockFromIsr();
  //  LEDON(2);
  ResetI2CTimer();
  AddLog(EVT_TIME_ISR, 0);

  switch(s_reg.stat) {
  case I2C_STAT_WAIT_STOP:
    AddLog(EVT_TIMEOUT1, 0);
    if( s_i2cp->i2c->CONSET & I2CF_STO ) 
      FinishI2C(E_I2C_TIMEOUT2);
    else 
      StartI2C();
    break;
  case I2C_STAT_WAIT_START:
    AddLog(EVT_TIMEOUT2, 0);
    if(s_reg.retry_start > 0) {
      s_reg.retry_start--;
      TryReStartI2C();
    }
    else 
      FinishI2C(E_I2C_TIMEOUT);
    break;
  case I2C_STAT_RUN:
    AddLog(EVT_TIMEOUT3, 0);
    if(s_reg.retry_start > 0) {
      s_reg.retry_start--;
      TryReStartI2C();
    }
    else 
      FinishI2C(E_I2C_TIMEOUT1);
    break;
  }

  chSysUnlockFromIsr();
}

// 输入: devAddr: 低7位是设备地址
//       bRead=1, 读，=0, 写
//       bStop=1, 完成后停止，=0, 完成后不停止
// 返回>=0, 读/写的字节数，<0, 错误
int Locked_I2C_Request(uint8_t devAddr, uint8_t *buf, int len, int bRead, int bStop, int retry, int needAck /* = 1 */)
{
  //  disable_interrupt();
  chSysLock();
#ifdef DEBUG_I2C
  s_log_idx=0;
#endif

  volatile uint32_t i = 0;
  s_reg.devAddr = devAddr;
  s_reg.buf = buf;
  s_reg.len = len;
  s_reg.bRead = bRead;
  s_reg.bStop = bStop;
  s_reg.retry_start = START_RETRY_NUM;
  s_reg.retry_run = retry;
  s_reg.needAck = needAck;
  //s_done = 0; 
  chBSemResetI(&s_done, TRUE);

  //启动i2c超时定时器
  //ResetI2CTimer();
  if (chVTIsArmedI(&s_vt))
    chVTResetI(&s_vt);
  chVTSetI(&s_vt, US2ST(I2C_RETRY_INTERVAL), i2c_tm_cb, NULL);
  //允许i2c超时定时器
  //EnableI2CTimer();

  TryStartI2C();
  chSysUnlock();
  //  enable_interrupt();

  //wait_flag(&s_done);
  int ret = chBSemWaitTimeoutS(&s_done, MS2ST(100));
  AddLog(EVT_RET, ret);

  ret = (s_reg.err<0) ? s_reg.err : s_reg.rwBytes;
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
static void serve_interrupt(uint8_t n) {
  (void)n;
  chSysLockFromIsr();
  int32_t err = 0;
  static uint32_t cnt = 0;
  ResetI2CTimer();

  uint8_t stat = s_i2cp->i2c->STAT; 
  AddLog(EVT_IIC_ISR, stat);

  switch(stat) {	
  case 0x00:
    err = E_I2C_BUS;
    break;

  case 0x08: //发送了START
  case 0x10: //发送了ReSTART
    LEDON(4);
    s_reg.stat = I2C_STAT_RUN;
    AddLog(EVT_DATA, (s_reg.devAddr<<1) | s_reg.bRead);
    /* send devAddr + R/W */
    s_i2cp->i2c->DAT = (s_reg.devAddr<<1) | s_reg.bRead;
    s_i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
    break;

  case 0x20: // 发送addr+W, 未收到ACK
  case 0x30: // 发送数据，未收到ACK
    LEDON(3);
    if(s_reg.needAck) {
      // 缺省处理. 
      err = E_I2C_NACK;
      break;
    }

  case 0x18: // 发送addr+W, 收到ACK
  case 0x28: // 发送数据，收到ACK
    LEDON(1);
    if(s_reg.rwBytes<s_reg.len) {
      AddLog(EVT_DATA, s_reg.buf[s_reg.rwBytes]);
      s_i2cp->i2c->DAT = s_reg.buf[s_reg.rwBytes++];
      s_i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
    }
    else
      err = 1; // 表示全部发送完毕
    break;

  case 0x38: //arbitration lost in SLA+R/W or data bytes
    err = E_I2C_ARB;
    break;

  case 0x48: // 发送了addr+R, 未收到ACK
    if(s_reg.needAck) {
      err = E_I2C_NACK;
      break;
    }

  case 0x40: //发送了addr+R, 收到ACK
    if(s_reg.len > 1) {
      /* Acknowledge the next received byte */
      s_i2cp->i2c->CONSET = I2CF_AA;
    }
    s_i2cp->i2c->CONCLR = I2CF_STA | I2CF_SI;
    break;

  case 0x50: // 收到数据，应答了ACK
    s_reg.buf[s_reg.rwBytes++] = (uint8_t)(s_i2cp->i2c->DAT);
    s_i2cp->i2c->CONCLR = (s_reg.rwBytes == s_reg.len-1) ? (I2CF_STA|I2CF_SI|I2CF_AA) : (I2CF_STA|I2CF_SI);
    break;

  case 0x58: // 收到数据，未应答ACK
    s_reg.buf[s_reg.rwBytes++] = (uint8_t)(s_i2cp->i2c->DAT);
    err = 1;  // 完成
    break;

  default:
    err = E_I2C_STAT;
  }

  if(err<0 && s_reg.retry_run>0) {
    AddLog(EVT_DATA, s_reg.retry_run);
    s_reg.retry_run--;
    s_reg.retry_start = START_RETRY_NUM;
    TryReStartI2C();
  }
  else 
    if(err) {
      FinishI2C(err);
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

  serve_interrupt(0);

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

  serve_interrupt(1);

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

  serve_interrupt(2);

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
  i2cObjectInit(&I2CD0);
  I2CD0.i2c = (LPC_I2C_TypeDef*) LPC_I2C0;
#endif /* LPC17xx_I2C_USE_I2C0 */

#if LPC17xx_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = (LPC_I2C_TypeDef*) LPC_I2C1;
#endif /* LPC17xx_I2C_USE_I2C1 */

#if LPC17xx_I2C_USE_I2C2
  i2cObjectInit(&I2CD2);
  I2CD2.i2c = (LPC_I2C_TypeDef*) LPC_I2C2;
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
  s_i2cp = i2cp;
  chBSemInit(&s_done, FALSE);

  if (i2cp->state == I2C_STOP) {
    /* Enables the peripheral.*/
#if LPC17xx_I2C_USE_I2C0
    if (&I2CD0 == i2cp) {
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
    if (&I2CD1 == i2cp) {
      LPC_SC->PCONP |=(1<<19);	                       //i2c1使能
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
    if (&I2CD2 == i2cp) {
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
    /*--- Reset registers ---*/
    i2cp->i2c->SCLL   = I2SCLL_SCLL;
    i2cp->i2c->SCLH   = I2SCLH_SCLH;
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
    if (&I2CD0 == i2cp) {

    }
#endif /* LPC17xx_I2C_USE_I2C0 */
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 * @details Number of receiving bytes must be more than 1 on STM32F1x. This is
 *          hardware restriction.
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

  (void)i2cp;
  int32_t ret = 0;
  static VirtualTimer vt;
  chVTSetI(&vt, timeout, dummy, NULL);
  while (TRUE) {
    ret = Locked_I2C_Request(addr, rxbuf, rxbytes, 
			     I2C_B_READ, I2C_B_STOP1,
			     2, I2C_B_NEEDACK);
    if (ret > 0) {
      return RDY_OK;
    }
    else {
      return RDY_RESET;
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

  (void)i2cp;
  uint32_t retry = timeout / I2C_RETRY_INTERVAL;
  int ret = Locked_I2C_Request(addr, txbuf, txbytes, I2C_B_WRITE, I2C_B_STOP0, retry, I2C_B_NEEDACK);
  if (ret == 1) {
    if (rxbytes > 0) {
      ret = Locked_I2C_Request(addr, rxbuf, rxbytes, I2C_B_READ, I2C_B_STOP1, 0, I2C_B_NEEDACK);
      if (ret > 0) {
	return RDY_OK;
      }
      else {
	return RDY_RESET;
      }
    }
    else {
      return RDY_OK;
    }
  }
  return RDY_RESET;
}

#endif /* HAL_USE_I2C */

/** @} */


