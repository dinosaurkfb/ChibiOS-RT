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
#define BOARD_NAME "Base Board + LPC1766"

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

#define LEDOFF(x)        (LPC_GPIO2->FIOSET0 |= 1ul << (x))
#define LEDON(x)         (LPC_GPIO2->FIOCLR0 |= 1ul << (x))

/*
 * Use 4-bits LED to show a number(0~15)
 */
void ShowLEDNum(uint32_t num);
void LongShowLEDNum(uint32_t num);

/*********************************************************************************************************
** 函数名称：myDelay
** 函数描述：软件延时
** 输入参数：ulTime    延时大小
** 返回值  ：无
*********************************************************************************************************/
void myDelay (uint32_t ulTime);

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

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
