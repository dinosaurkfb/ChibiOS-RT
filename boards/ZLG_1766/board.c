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

#include "ch.h"
#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {
 {VAL_GPIO0DATA, VAL_GPIO0DIR},
 {VAL_GPIO1DATA, VAL_GPIO1DIR},
 {VAL_GPIO2DATA, VAL_GPIO2DIR},
 {VAL_GPIO3DATA, VAL_GPIO3DIR},
};
#endif

/**
 * @brief   Delay a specific milliseconds approximately, used before timer is
 *          set up.
 *
 * @param[in] ms        milliseconds to delay.
 *
 * @notapi
 */
void myDelay (uint32_t ms)
{
    volatile uint32_t i = 0;

    while (ms--) {
        for (i = 0; i < 20000; i++);
    }
}


/**
 * @brief   GPIO initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
void GPIOInit( void )
{
    LPC_GPIO3->FIODIR |= 1<<30;                                            /* 设置P3.30为输出              */
    LPC_GPIO3->FIOSET |= 1<<30;                                            /* 设置P3.30为高电平            */
    LPC_GPIO2->FIODIR0 |= 1<<0;                                            /* 设置P2.0为输出              */
    LPC_GPIO2->FIOSET0 |= 1<<0;                                            /* 设置P2.0为高电平            */

    LPC_GPIO2->FIODIR0 |= 1<<1;                                            /* 设置P2.0为输出              */
    LPC_GPIO2->FIOSET0 |= 1<<1;                                            /* 设置P2.0为高电平            */

    LPC_GPIO2->FIODIR0 |= 1<<2;                                            /* 设置P2.0为输出              */
    LPC_GPIO2->FIOSET0 |= 1<<2;                                            /* 设置P2.0为高电平            */

    LPC_GPIO2->FIODIR0 |= 1<<3;                                            /* 设置P2.0为输出              */
    LPC_GPIO2->FIOSET0 |= 1<<3;                                            /* 设置P2.0为高电平            */
}

void _ShowLEDNum(uint32_t num) {
  
  volatile uint32_t i = 0;
  for (i = 0; i < 4; i++) {
    if (num & (1 << i)) {
      LEDON(i);
    } else {
      LEDOFF(i);
    }
  }
}

void ShowLEDNum(uint32_t num) {

  _ShowLEDNum(num);
  myDelay(100);
  _ShowLEDNum(0);
  myDelay(100);
  _ShowLEDNum(num);
  myDelay(100);
  _ShowLEDNum(0);
  myDelay(400);
}

void LongShowLEDNum(uint32_t num) {

  _ShowLEDNum(num);
  myDelay(800);
  _ShowLEDNum(0);
  /* myDelay(100); */
  /* _ShowLEDNum(num); */
  /* myDelay(100); */
  /* _ShowLEDNum(0); */
  myDelay(400);
}


/*********************************************************************************************************
** 函数名称：LEDOperate
** 函数描述：LED工作
** 输入参数：无
** 返回值  ：无
*********************************************************************************************************/
void LEDOperate(void)
{
  _ShowLEDNum(0xF);
  myDelay(100);

  _ShowLEDNum(0x0);
  myDelay(100);

  _ShowLEDNum(0xF);
  myDelay(100);

  _ShowLEDNum(0x0);
  myDelay(100);
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
  LPC17xx_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
  GPIOInit();
  LEDOperate();
}
