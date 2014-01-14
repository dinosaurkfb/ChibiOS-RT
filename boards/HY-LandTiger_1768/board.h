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
 * Setup for Embedded HY-LandTiger LPC1768 board.
 */

/*
 * Board identifiers.
 */
#define BOARD_LPC1768
#define BOARD_NAME "Base Board + LPC1768"

/*
 * Board frequencies.
 */
#define MAINOSCCLK              12000000

/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            PAL_PORT_BIT(GPIO0_OLEDSEL) |               \
                                PAL_PORT_BIT(GPIO0_LED2)
#define VAL_GPIO0DATA           PAL_PORT_BIT(GPIO0_OLEDSEL) |               \
                                PAL_PORT_BIT(GPIO0_LED2)

#define LEDOFF(x)               (LPC_GPIO2->FIOSET0 |= 1ul << (x))
#define LEDON(x)                (LPC_GPIO2->FIOCLR0 |= 1ul << (x))


/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            PAL_PORT_BIT(GPIO1_LED3B)   |               \
                                PAL_PORT_BIT(GPIO1_LED3R)   |               \
                                PAL_PORT_BIT(GPIO1_LED3G)   |               \
                                PAL_PORT_BIT(GPIO1_SPI0SEL)
#define VAL_GPIO1DATA           PAL_PORT_BIT(GPIO1_LED3B)   |               \
                                PAL_PORT_BIT(GPIO1_LED3R)   |               \
                                PAL_PORT_BIT(GPIO1_LED3G)   |               \
                                PAL_PORT_BIT(GPIO1_SPI0SEL)

/*
 * GPIO 2 initial setup.
 */
#define VAL_GPIO2DIR            0x00000000
#define VAL_GPIO2DATA           0x00000000

/*
 * GPIO 3 initial setup.
 */
#define VAL_GPIO3DIR            0x00000000
#define VAL_GPIO3DATA           0x00000000

/*
 * Pin definitions.
 */
#define GPIO0_SW3               1
#define GPIO0_OLEDSEL           2
#define GPIO0_LED2              7

#define GPIO1_LED3B             2
#define GPIO1_SW4               4
#define GPIO1_LED3R             9
#define GPIO1_LED3G             10
#define GPIO1_SPI0SEL           11

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
