#ifndef _UPDATE_H_
#define _UPDATE_H_

#if ENABLE_IAP

#define TYPE_UART_UPDATA      0x20
#define TYPE_UART_WINFO       0x00
#define TYPE_UART_RINFO       0x01
#define TYPE_UART_DEBUG       0x02
#define TYPE_UART_ERR         0x03
#define TYPE_UART_TIME        0x04
#define TYPE_UART_PECUID      0x05

#define TYPE_UART_RINFO_ACK   0x81


#define E2PROM_EQUID_ADDR     0x00
#define EQUID_LEN             0x07
#define E2PROM_DEBUG_ADDR     0x08

#define UPDATE_TYPE  0x50414920

#define CHIP_TYPE 1788

/* Board hardware version */
#define HW_VERSION 0x01

#if !defined(SW_VERSION)
#define SW_VERSION 0x01
#endif

/*
 * bits 8~15: board hardware version
 * bits 0~7:  board software version
 */
#define CODE_VERSION ((HW_VERSION) << 8 | (SW_VERSION))

#define COM0_BAUD_RATE 115200

#define ISP_BAUD_RATE 115200

#define OSC_FREQ_KHZ 12000

/* boot code should be cut down to less then 8k */
#define IAP_SEC_FLAG_ADDR 0x00007000	/* 28k */

/* start address of user code */
#define IAP_SEC_USER_ADDR 0x00008000	/* 32k */

#define IAP_BAUD1_RATE USE_BTR_CAN1
#define IAP_BAUD2_RATE USE_BTR_CAN2

typedef void (* FUNC_APP_START)(void); 
extern const uint32_t update_cfg[12];
extern const void *app_addr;
extern FUNC_APP_START app_start;
extern uint8_t * g_p_app_flag;

#ifdef __cplusplus
extern "C" {
#endif
  void uart0_scan(void);
#ifdef __cplusplus
}
#endif

#endif /* #if ENABLE_IAP */
#endif /* _UPDATE_H_ */
