#ifndef _AT24C0X_H_
#define _AT24C0X_H_

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void EEPROMInit(I2CDriver *i2cp);
  int at24c0x_write_byte(uint8_t addr, uint8_t byte);
  int at24c0x_random_read(uint8_t addr, uint8_t *rxbuf);
  int at24c0x_cur_read(uint8_t *rxbuf);
  int WriteEEPROM(uint8_t addr, uint8_t *buf, size_t len);
  int ReadEEPROM(uint8_t addr, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _AT24C0X_H_ */
