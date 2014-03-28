#include "ch.h"
#include "hal.h"
#include "string.h"
#include "mc24lc0x.h"
#include "lpc_types.h"

#if HAL_USE_I2C
#if EEPROM_USE_MC24LC0X
static I2CDriver *s_i2cp;

//Mc24lc0x requires a 0x1010xxx as a slave address
#define dev_addr   0b1010000
static uint8_t s_buf[9];

/* I2C interface config */
static const I2CConfig i2ccfg = {
  I2C_STANDARD_MODE,
  400000
};

void EEPROMInit(I2CDriver *i2cp, const I2CConfig *i2ccfgp) {
  /*
   * Starts I2C
   */
  s_i2cp = i2cp;
  i2ccfgp == NULL ? i2cStart(i2cp, &i2ccfg) : i2cStart(i2cp, i2ccfgp);
}

msg_t mc24lc0x_write_byte(I2CDriver *i2cp, uint32_t laddr, uint8_t byte) {
  /* mc24lc0x has 8 bits internal address */
  uint8_t addr = (uint8_t)laddr;
  uint8_t buf[2] = {addr, byte};
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;
  uint32_t retry = 5;

  i2cAcquireBus(i2cp);
  for (i = 1; i <= retry; i++) {
    ret = i2cMasterTransmitTimeout(i2cp, dev_addr, buf, 2, NULL, 0, MS2ST(1));
    if (ret == RDY_RESET) {
      /* Reset I2C and try again. */
      i2cStop(s_i2cp);
      chThdSleepMilliseconds(2);
      i2cStart(s_i2cp, s_i2cp->config);
      continue;
      }
    else {
      break;
    }
  }
  i2cReleaseBus(i2cp);
  /* if (ret != RDY_OK) { */
  /*   I2C_Dump(); */
  /* } */
  return ret;
}

msg_t mc24lc0x_cur_read(I2CDriver *i2cp, uint8_t *rxbuf) {
  /* mc24lc0x has 8 bits internal address */
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;
  uint32_t retry = 5;

  i2cAcquireBus(i2cp);
  for (i = 1; i <= retry; i++) {
    ret = i2cMasterReceiveTimeout(i2cp, dev_addr, rxbuf, 1, MS2ST(4));
    if (ret == RDY_RESET) {
      /* Reset I2C and try again. */
      i2cStop(s_i2cp);
      chThdSleepMilliseconds(2);
      i2cStart(s_i2cp, s_i2cp->config);
      continue;
      }
    else {
      break;
    }
  }
  i2cReleaseBus(i2cp);
  /* if (ret != RDY_OK) { */
  /*   LOG_PRINT("ret = %d\n", ret); */
  /*   I2C_Dump(); */
  /* } */
  return ret;
}

msg_t mc24lc0x_random_read(I2CDriver *i2cp, uint32_t laddr, uint8_t *rxbuf) {
  uint8_t addr = (uint8_t)laddr;
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;
  uint32_t retry = 5;

  i2cAcquireBus(i2cp);
  for (i = 1; i <= retry; i++) {
    ret = i2cMasterTransmitTimeout(i2cp, dev_addr, &addr, 1, rxbuf, 1, MS2ST(4));
    if (ret == RDY_RESET) {
      /* Reset I2C and try again. */
      i2cStop(s_i2cp);
      chThdSleepMilliseconds(2);
      i2cStart(s_i2cp, s_i2cp->config);
      continue;
      }
    else {
      break;
    }
  }
  i2cReleaseBus(i2cp);
  return ret;
}

int WriteEEPROM(uint32_t laddr, uint8_t *buf, size_t len)
{
  /* mc24lc0x has 8 bits internal address */
  uint8_t addr = (uint8_t)laddr;
  uint8_t offset = 0;
  size_t bytes_left = len;
  size_t bytes_once = 8;
  uint32_t retry = 5;
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;
  len = MIN(len, EEPROM_SIZE);

  i2cAcquireBus(s_i2cp);
  while (bytes_left > 0) {
    s_buf[0] = addr + offset;
    if (bytes_left < 8) {
      bytes_once = bytes_left;
    }
    memcpy(&s_buf[1], buf + offset, bytes_once);
    ret = RDY_RESET;
    for (i = 1; i <= retry; i++) {
      ret = i2cMasterTransmitTimeout(s_i2cp, dev_addr,
                                     s_buf, bytes_once+1,
                                     NULL, 0,
                                     MS2ST(4));
      if (ret == RDY_RESET) {
        /* Reset I2C and try again. */
        i2cStop(s_i2cp);
        chThdSleepMilliseconds(2);
        i2cStart(s_i2cp, s_i2cp->config);
        continue;
      }
      else {
        break;
      }
    }
    if (ret == RDY_OK) {
      for (i = 0; i < 10000; i++);
      offset += bytes_once;
      bytes_left -= bytes_once;
    } else {
      break;
    }
  }
  i2cReleaseBus(s_i2cp);
  return ret;
}

// len <= 256
// 返回>=0, 读/写的字节数，<0, 错误
int ReadEEPROM(uint32_t laddr, uint8_t *buf, size_t len)
{
  /* mc24lc0x has 8 bits internal address */
  uint8_t addr = (uint8_t)laddr;
  i2cAcquireBus(s_i2cp);
  uint32_t retry = 5;
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;
  len = MIN(len, EEPROM_SIZE);

  for (i = 1; i <= retry; i++) {
    ret = i2cMasterTransmitTimeout(s_i2cp, dev_addr,
                                   &addr, 1,
                                   buf, len,
                                   MS2ST(10));
    if (ret == RDY_RESET) {
      /* Reset I2C and try again. */
      i2cStop(s_i2cp);
      chThdSleepMilliseconds(2);
      i2cStart(s_i2cp, s_i2cp->config);
      /* LOG_PRINT("Retry %d\n", i); */
      continue;
      }
    else {
      break;
    }
  }
  i2cReleaseBus(s_i2cp);
  /* if (ret != RDY_OK) { */
  /*   LOG_PRINT("ret = %d\n", ret); */
  /*   I2C_Dump(); */
  /* } */
  return ret;
}

#endif /* #if EEPROM_USE_MC24LC0X */
#endif /* #if HAL_USE_I2C */
