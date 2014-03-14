#ifndef _UPDATE_H_
#define _UPDATE_H_

/**
 * @brief   Serial driver definition when using Serial to update program.
 * @note    For some platforms, there is only one uart port, and for platforms
 *          like the LPC series, ISP is only supported through uart0. So,
 *          considering the generality, only SD1 is supported as an updating
 *          Serial Driver currently. 
 */
#if !defined(updateSD) || defined(__DOXYGEN__)
#define updateSD                    SD1
#endif

#if (updateSD != SD1)
#error "Serial updating support only through SD1."
#endif


#if ENABLE_IAP

/**
 * @brief   CAN driver definition when using CAN to update program.
 * @note    For some platforms, there is only one CAN port. So, considering
 *          the generality, only CAND1 is supported as an updating CAN
 *          Driver currently. 
 */
#if !defined(updateCAND) || defined(__DOXYGEN__)
#define updateCAND                  CAND1
#endif

#if (updateCAND != CAND1)
#error "CAN updating support only through CAND1."
#endif

#endif /* #if ENABLE_IAP */

#define TYPE_UART_UPDATA            0x20
#define TYPE_UART_WINFO             0x00
#define TYPE_UART_RINFO             0x01
#define TYPE_UART_DEBUG             0x02
#define TYPE_UART_ERR               0x03
#define TYPE_UART_TIME              0x04
#define TYPE_UART_PECUID            0x05

#define TYPE_UART_RINFO_ACK         0x81


#define E2PROM_EQUID_ADDR           0x00
#define EQUID_LEN                   0x07
#define E2PROM_DEBUG_ADDR           0x08

/**
 * @brief   CHIP_TYPE definition.
 * @note    This value is checked by updating program to see if the HEX file
 *          is valid.
 */
#if !defined(CHIP_TYPE) || defined(__DOXYGEN__)
#define CHIP_TYPE                   1766
#endif

/**
 * @brief   Board hardware version definition.
 * @note    This value is checked by updating program to see if the HEX file
 *          is valid.
 */
#if !defined(HW_VERSION) || defined(__DOXYGEN__)
#define HW_VERSION                  0x01
#endif

/**
 * @brief   Board software version definition.
 * @note    This value is checked by updating program to see if the HEX file
 *          is valid.
 */
#if !defined(SW_VERSION) || defined(__DOXYGEN__)
#define SW_VERSION                  0x01
#endif

/**
 * @brief   Board hardware and software version definition.
 * @note    This value is checked by updating program to see if the HEX file
 *          is valid.
 * @note    bits 8~15: board hardware version
 *          bits 0~7:  board software version
 */
#define CODE_VERSION ((HW_VERSION) << 8 | (SW_VERSION))

/**
 * @brief   The baudrate setting of a running Serial driver.
 * @note    This module rely on a running Serial driver and a pre-defined
 *          UPDATE_BAUD_RATE before using this module. Normally, define it in
 *          your board.h.
 *          
 */
#if !defined(UPDATE_BAUD_RATE) || defined(__DOXYGEN__)
#define UPDATE_BAUD_RATE            9600
#endif

/**
 * @brief   The baudrate setting of ISP.
 * @note    This value rely on platforms and CCLK. Normally, define it in
 *          your board.h.
 */
#if !defined(ISP_BAUD_RATE) || defined(__DOXYGEN__)
#define ISP_BAUD_RATE               9600
#endif

#define OSC_FREQ_KHZ                (MAINOSCCLK/1000)

#if ENABLE_IAP
#define UPDATE_TYPE                 0x50414920

/* boot code should be cut down to less then 8k */
#define IAP_SEC_FLAG_ADDR           0x00007000	/* 28k */

/* start address of user code */
#define IAP_SEC_USER_ADDR           0x00008000	/* 32k */

#define IAP_BAUD1_RATE              USE_BTR_CAN1
#define IAP_BAUD2_RATE              USE_BTR_CAN2
#endif /* #if ENABLE_IAP */


#if ENABLE_IAP
extern const uint32_t update_cfg[12];
#else
extern const uint32_t update_cfg[9];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void updateThreadStart(void);
#ifdef __cplusplus
}
#endif

#endif /* _UPDATE_H_ */
