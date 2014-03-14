#ifndef _IAP_H_
#define _IAP_H_

#include "ch.h"
#include "hal.h"
/* Output parameters for IAP */
extern uint32_t  paramout[8];

/* ISP Return Codes */
#define  CMD_SUCCESS          0
#define  INVALID_COMMAND      1
#define  SRC_ADDR_ERROR       2 
#define  DST_ADDR_ERROR       3
#define  SRC_ADDR_NOT_MAPPED  4
#define  DST_ADDR_NOT_MAPPED  5
#define  COUNT_ERROR          6
#define  INVALID_SECTOR       7
#define  SECTOR_NOT_BLANK     8
#define  SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9
#define  COMPARE_ERROR        10
#define  BUSY                 11
#define  PARAM_ERROR          12 /* Insufficient number of parameters */
#define  ADDR_ERROR           13 /* Address not on word boundary */
#define  ADDR_NOT_MAPPED      14
#define  CMD_LOCKED           15 /* Command is locked */
#define  INVALID_CODE         16 /* Unlock code is invalid */
#define  INVALID_BAUD_RATE    17
#define  INVALID_STOP_BIT     18 

#ifdef __cplusplus
extern "C" {
#endif
  void SelSector(uint8_t sec1, uint8_t sec2);
  void RamToFlash(uint32_t dst, uint32_t src, uint32_t no);
  void EraseSector(uint8_t sec1, uint8_t sec2);
  void BlankCHK(uint8_t sec1, uint8_t sec2);
  void ReadParID(void);
  void BootCodeID(void);
  void Compare(uint32_t dst, uint32_t src, uint32_t no);
  void enter_isp_with_wdt(void);
#ifdef __cplusplus
}
#endif

#endif /* _IAP_H_ */
