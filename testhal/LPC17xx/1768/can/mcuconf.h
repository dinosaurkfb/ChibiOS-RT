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

/*
 * LPC17xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the driver
 * is enabled in halconf.h.
 *
 * IRQ priorities:
 * 7...0        Lowest...highest.
 */

/*
 * HAL driver system settings.
 */
#define LPC17xx_MAINOSC_ENABLE              TRUE
#define LPC17xx_SYSCLK_SELECT               CLKSRCSEL_MAINOSC
#define LPC17xx_MAINPLL_ENABLE              TRUE
#define LPC17xx_MAINPLL_MUL                 100
#define LPC17xx_MAINPLL_PREDIV              3
#define LPC17xx_USBPLL_ENABLE               FALSE
#define LPC17xx_USBPLL_MUL                  4
#define LPC17xx_USBPLL_DIV                  4
#define LPC17xx_CCLK_DIV                    5
#define LPC17xx_PCLK_SELECT                 PCLKSEL_CCLK
#define LPC17xx_CLKOUT_ENABLE               FALSE
#define LPC17xx_CLKOUT_DIV                  4
#define LPC17xx_CLKOUT_SELECT               CLKOUTSEL_MAINOSC

/*
 * GPT driver system settings.
 */
#define LPC17xx_GPT_USE_TIM0                TRUE
#define LPC17xx_GPT_USE_TIM1                TRUE
#define LPC17xx_GPT_USE_TIM2                TRUE
#define LPC17xx_GPT_USE_TIM3                TRUE
#define LPC17xx_GPT_TIM0_IRQ_PRIORITY       2
#define LPC17xx_GPT_TIM1_IRQ_PRIORITY       6
#define LPC17xx_GPT_TIM2_IRQ_PRIORITY       2
#define LPC17xx_GPT_TIM3_IRQ_PRIORITY       2

/*
 * SERIAL driver system settings.
 */
#define LPC17xx_SERIAL_USE_UART0            TRUE
#define LPC17xx_SERIAL_FIFO_PRELOAD         16
#define LPC17xx_SERIAL_UART0_IRQ_PRIORITY   3

/*
 * RTC driver system settings.
 */
#define LPC17xx_RTC_IS_CALENDAR             TRUE
#define LPC17xx_RTC_USE_ALARM               TRUE
#define LPC17xx_RTC_IRQ_PRIORITY            0

/*
 * CAN driver system settings.
 */
#define LPC17XX_CAN_USE_CAN1                  TRUE
#define LPC17XX_CAN_CAN1_IRQ_PRIORITY         11

#define LPC17XX_CAN_USE_CAN2                  TRUE
#define LPC17XX_CAN_CAN2_IRQ_PRIORITY         11

/* The length of RX buffer (Must be a multiple of 2) */
#define LPC17XX_CAN_RX_BUF_NUM				  32					
