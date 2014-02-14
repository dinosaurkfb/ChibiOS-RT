#include "ch.h"
#include "hal.h"
#include "string.h"

#if HAL_USE_I2C
static I2CDriver *s_i2cp;

//AT24C0x requires a 0x1010xxx as a slave address
#define dev_addr   0b1010000
static uint8_t s_buf[9];

void EEPROMInit(I2CDriver *i2cp) {
  /*
   * Starts I2C
   */
  s_i2cp = i2cp;
  i2cStart(i2cp, NULL);
}

msg_t at24c0x_write_byte(I2CDriver *i2cp, uint8_t addr, uint8_t byte) {
  uint8_t buf[2] = {addr, byte};
  msg_t ret = i2cMasterTransmitTimeout(i2cp, dev_addr, buf, 2, NULL, 0, MS2ST(1));
  return ret;
}

msg_t at24c0x_cur_read(I2CDriver *i2cp, uint8_t *rxbuf) {
  msg_t ret = i2cMasterReceiveTimeout(i2cp, dev_addr, rxbuf, 1, MS2ST(1));
  return ret;
}

int _at24c0x_cur_read(I2CDriver *i2cp, uint8_t *rxbuf) {
  int32_t i, ret = 0;

  for (i = 5; i > 0; i--) {
    ret = Locked_I2C_Request(i2cp, dev_addr, rxbuf, 1, I2C_B_READ, I2C_B_STOP1, 2, I2C_B_NEEDACK);
    if (ret == 0) {
      break;
    }
  }

  return ret;
}

msg_t at24c0x_random_read(I2CDriver *i2cp, uint8_t addr, uint8_t *rxbuf) {
  /* Write data word address first */
  msg_t ret = i2cMasterTransmitTimeout(i2cp, dev_addr, &addr, 1, rxbuf, 1, MS2ST(1));
  return ret;
}

int WriteEEPROM(uint8_t addr, uint8_t *buf, size_t len)
{
  uint8_t offset = 0;
  size_t bytes_left = len;
  size_t bytes_once = 8;
  msg_t ret = RDY_OK;
  volatile uint32_t i = 0;

  i2cAcquireBus(s_i2cp);
  while (bytes_left > 0) {
    s_buf[0] = addr + offset;
    if (bytes_left < 8) {
      bytes_once = bytes_left;
    }
    memcpy(&s_buf[1], buf + offset, bytes_once);
    ret = i2cMasterTransmitTimeout(s_i2cp, dev_addr,
				   s_buf, bytes_once+1,
				   NULL, 0,
				   MS2ST(2));
    if (ret == RDY_OK) {
      for (i = 0; i < 20000; i++);
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
int ReadEEPROM(uint8_t addr, uint8_t *buf, size_t len)
{
  i2cAcquireBus(s_i2cp);
  msg_t ret= i2cMasterTransmitTimeout(s_i2cp, dev_addr,
				      &addr, 1,
				      buf, len,
				      MS2ST(2));
  i2cReleaseBus(s_i2cp);
  return ret;
}

#endif /* #if HAL_USE_I2C */
