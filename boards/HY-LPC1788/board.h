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
 * Setup for Embedded HY-LandTiger LPC1788 board.
 */

/*
 * Board identifiers.
 */
#define BOARD_LPC1788
#define LPC177x_8x
#define BOARD_NAME "HY-LPC1788 Board with LPC1788 chip."

/*
 * Board frequencies.
 */
#define MAINOSCCLK              12000000

/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            0x00000000
#define VAL_GPIO0DATA           0x00000000

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            PAL_PORT_BIT(GPIO1_LD7)
#define VAL_GPIO1DATA           PAL_PORT_BIT(GPIO1_LD7)

/*
 * GPIO 2 initial setup.
 */
#define VAL_GPIO2DIR            PAL_PORT_BIT(GPIO2_LD2)
#define VAL_GPIO2DATA           PAL_PORT_BIT(GPIO2_LD2)
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
 * GPIO 5 initial setup.
 */
#define VAL_GPIO5DIR            PAL_PORT_BIT(GPIO5_LD8)    |               \
                                PAL_PORT_BIT(GPIO5_LD9)
#define VAL_GPIO5DATA           PAL_PORT_BIT(GPIO5_LD8)    |               \
                                PAL_PORT_BIT(GPIO5_LD9)
/*
 * Pin definitions.
 */
/* Note: Both LD2 in the core board and LD6 in the SDK board connect to P2.21 */
/* LD2 in the core board connects to P2.21 */
#define GPIO2_LD2               21

/* LED1 in SDK board connects to P2.21 */
#define GPIO2_LD6               21

/* LED2 in SDK board connects to P1.13 */
#define GPIO1_LD7               13

/* LED3 in SDK board connects to P5.0 */
#define GPIO5_LD8               0

/* LED4 in SDK board connects to P5.1 */
#define GPIO5_LD9               1


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
  void LEDON(uint8_t x);
  void LEDOFF(uint8_t x);
  void ToggleLED(uint8_t x);
  void ledSingleBlinkBin(uint32_t num, uint32_t interval);
  void ledDoubleBlinkBin(uint32_t num, uint32_t interval);
  void LOG_PRINT(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
