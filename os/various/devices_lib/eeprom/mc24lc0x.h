#ifndef _MC24LC0X_H_
#define _MC24LC0X_H_
#if HAL_USE_I2C
#if EEPROM_USE_MC24LC0X

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void EEPROMInit(I2CDriver *i2cp, const I2CConfig *i2ccfgp);
  msg_t mc24lc0x_write_byte(I2CDriver *i2cp, uint8_t addr, uint8_t byte);
  msg_t mc24lc0x_random_read(I2CDriver *i2cp, uint8_t addr, uint8_t *rxbuf);
  msg_t mc24lc0x_cur_read(I2CDriver *i2cp, uint8_t *rxbuf);
  int WriteEEPROM(uint8_t addr, uint8_t *buf, size_t len);
  int ReadEEPROM(uint8_t addr, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* #if EEPROM_USE_MC24LC0X */
#endif /* #if HAL_USE_I2C */
#endif /* _MC24LC0X_H_ */
