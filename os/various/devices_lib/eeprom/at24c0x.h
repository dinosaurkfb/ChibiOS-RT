#ifndef _AT24C0X_H_
#define _AT24C0X_H_

#if HAL_USE_I2C
#if EEPROM_USE_AT24C0X
#define EEPROM_SIZE 256

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void EEPROMInit(I2CDriver *i2cp, const I2CConfig *i2ccfgp);
  msg_t at24c0x_write_byte(I2CDriver *i2cp, uint32_t laddr, uint8_t byte);
  msg_t at24c0x_random_read(I2CDriver *i2cp, uint32_t laddr, uint8_t *rxbuf);
  msg_t at24c0x_cur_read(I2CDriver *i2cp, uint8_t *rxbuf);
  int WriteEEPROM(uint32_t laddr, uint8_t *buf, size_t len);
  int ReadEEPROM(uint32_t laddr, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* #if EEPROM_USE_AT24C0X */
#endif /* #if HAL_USE_I2C */
#endif /* _AT24C0X_H_ */
