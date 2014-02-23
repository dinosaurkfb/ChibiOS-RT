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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Embedded Artists LPCXpresso Base Board with LPC1766 daughter
 * board.
 */

/*
 * Board identifiers.
 */
#define BOARD_LPC1766
#define BOARD_NAME "ZLG Board with LPC1766 chip."

/*
 * Board frequencies.
 */
#define MAINOSCCLK              12000000

/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            0x00000000
#define VAL_GPIO0DATA           0x00000000

#define LEDOFF(x)               palSetPad(GPIO2, (x - 1))
#define LEDON(x)                palClearPad(GPIO2, (x - 1))

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            0x00000000
#define VAL_GPIO1DATA           0x00000000

/*
 * GPIO 2 initial setup.
 */
#define VAL_GPIO2DIR            PAL_PORT_BIT(GPIO2_LED1)   |               \
                                PAL_PORT_BIT(GPIO2_LED2)   |               \
                                PAL_PORT_BIT(GPIO2_LED3)   |               \
                                PAL_PORT_BIT(GPIO2_LED4)

#define VAL_GPIO2DATA           PAL_PORT_BIT(GPIO2_LED1)   |               \
                                PAL_PORT_BIT(GPIO2_LED2)   |               \
                                PAL_PORT_BIT(GPIO2_LED3)   |               \
                                PAL_PORT_BIT(GPIO2_LED4)
/*
 * GPIO 3 initial setup.
 */
#define VAL_GPIO3DIR            0x00000000
#define VAL_GPIO3DATA           0x00000000

/*
 * GPIO 4 initial setup.
 */
#define VAL_GPIO4DIR            0x00000000
#define VAL_GPIO4DATA           0x00000000

/*
 * Pin definitions.
 */
#define GPIO2_LED1              0
#define GPIO2_LED2              1
#define GPIO2_LED3              2
#define GPIO2_LED4              3

#define CHIP_TYPE                   1766
#define UPDATE_BAUD_RATE            115200
#define ISP_BAUD_RATE               115200

/**
 * @brief   UART0 LOG_PRINT enable switch.
 * @details If set to @p TRUE LOG_PRINT via UART0 is supported.
 * @note    The default is @p TRUE .
 */
#if !defined(LOG_PRINT_USE_UART0) || defined(__DOXYGEN__)
#define LOG_PRINT_USE_UART0              TRUE
#endif


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void msDelay (uint32_t ulTime);
  void ledSingleBlinkBin(uint32_t num, uint32_t interval);
  void ledDoubleBlinkBin(uint32_t num, uint32_t interval);
  void LOG_PRINT(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
