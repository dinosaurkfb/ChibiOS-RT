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
#include "chprintf.h"
#include "pinsel_lld.h"

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
  {VAL_GPIO4DATA, VAL_GPIO4DIR},
  {VAL_GPIO5DATA, VAL_GPIO5DIR},
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
void msDelay (uint32_t ms) {

  volatile uint32_t i = 0;
  while (ms--) {
    for (i = 0; i < 20000; i++);
  }
}


/**
 * @brief   Turn off a LED.
 *
 * @param[in] x        Number of LED to turned off.
 *
 * @notapi
 */
void LEDOFF(uint8_t x) {
  switch (x) {
    case 1:
      palSetPad(GPIO2, GPIO2_LD6);
      break;
    case 2:
      palSetPad(GPIO1, GPIO1_LD7);
      break;
    case 3:
      palSetPad(GPIO5, GPIO5_LD8);
      break;
    case 4:
      palSetPad(GPIO5, GPIO5_LD9);
      break;
    default:
      break;
  }
}

/**
 * @brief   Turn on a LED.
 *
 * @param[in] x        Number of LED to turned on.
 *
 * @notapi
 */
void LEDON(uint8_t x) {
  switch (x) {
    case 1:
      palClearPad(GPIO2, GPIO2_LD6);
      break;
    case 2:
      palClearPad(GPIO1, GPIO1_LD7);
      break;
    case 3:
      palClearPad(GPIO5, GPIO5_LD8);
      break;
    case 4:
      palClearPad(GPIO5, GPIO5_LD9);
      break;
    default:
      break;
  }
}

/**
 * @brief   Toggle a LED.
 *
 * @param[in] x        Number of LED to toggled.
 *
 * @notapi
 */
void ToggleLED(uint8_t x) {
  switch (x) {
    case 1:
      palTogglePad(GPIO2, GPIO2_LD6);
      break;
    case 2:
      palTogglePad(GPIO1, GPIO1_LD7);
      break;
    case 3:
      palTogglePad(GPIO5, GPIO5_LD8);
      break;
    case 4:
      palTogglePad(GPIO5, GPIO5_LD9);
      break;
    default:
      break;
  }
}

/**
 * @brief   Use four leds to show a number in binary format.
 *
 * @param[in] mum       Number to be shown.
 *
 * @api
 */
void _ledShowBin(uint32_t num) {
  num & PAL_PORT_BIT(0) ? LEDON(1) : LEDOFF(1);
  num & PAL_PORT_BIT(1) ? LEDON(2) : LEDOFF(2);
  num & PAL_PORT_BIT(2) ? LEDON(3) : LEDOFF(3);
  num & PAL_PORT_BIT(3) ? LEDON(4) : LEDOFF(4);
}

/**
 * @brief   Double-blink four leds to denote a number in binary format.
 * @note    This function uses @p msDelay() to form a interval.
 *
 * @param[in] mum       Number to be blinked.
 * @param[in] interval  Interval time  when blinking.
 *
 * @api
 */
void ledDoubleBlinkBin(uint32_t num, uint32_t interval) {

  ledSingleBlinkBin(num, interval);
  ledSingleBlinkBin(num, interval);
}

/**
 * @brief   Single-blink four leds to denote a number in binary format.
 * @note    This function uses @p msDelay() to form a interval.
 *
 * @param[in] mum       Number to be blinked.
 * @param[in] interval  Interval time  when blinking.
 *
 * @api
 */
void ledSingleBlinkBin(uint32_t num, uint32_t interval) {

  _ledShowBin(num);
  msDelay(interval);
  _ledShowBin(0);
  msDelay(interval);
}


/**
 * @brief   Operate all leds as a test.
 *
 */
void ledOperate(void)
{
  _ledShowBin(0x1);
  msDelay(80);
  _ledShowBin(0x3);
  msDelay(80);
  _ledShowBin(0x7);
  msDelay(80);
  ledDoubleBlinkBin(0xF, 40);
}

void memdump(uint8_t *buf, size_t len)
{
  LOG_PRINT("###################################\n");
  uint32_t i = 0;
  for (i=0; i<len; ++i) {
    LOG_PRINT("%02x ", buf[i]);
    if (i % 16 == 15)
      LOG_PRINT("\n");
  }
  LOG_PRINT("\n");
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
  LPC17xx_clock_init();
}

/** @brief Driver default configuration.*/
static const SerialConfig uart0_config = {
  UPDATE_BAUD_RATE,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p printf() like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] fmt       formatting string
 *
 * @api
 */
void LOG_PRINT(const char *fmt, ...) {
  va_list ap;

  va_start(ap, fmt);
#if LOG_PRINT_USE_UART0
  chvprintf((BaseSequentialStream  *)&SD1, fmt, ap);
#endif
  va_end(ap);
}


/*
 * Board-specific initialization code.
 */
void boardInit(void) {
  ledOperate();
  /* set PIO0.27 and PIO0.28 to I2C0 SDA and SCL */
#if LPC17xx_I2C_USE_I2C0
  PINSEL_ConfigPin(0, 27, 0b001);
  PINSEL_ConfigPin(0, 28, 0b001);
#endif /* LPC17xx_I2C_USE_I2C0 */

#if LPC17xx_I2C_USE_I2C1
  /* set PIO2.14 and PIO2.15 to I2C1 SDA and SCL */
  PINSEL_ConfigPin(2, 14, 0b010);
  PINSEL_ConfigPin(2, 15, 0b010);
#endif /* LPC17xx_I2C_USE_I2C1 */

#if LPC17xx_I2C_USE_I2C2
  /* set PIO2.30 and PIO2.31 to I2C2 SDA and SCL */
  PINSEL_ConfigPin(2, 30, 0b010);
  PINSEL_ConfigPin(2, 31, 0b010);
#endif /* LPC17xx_I2C_USE_I2C2 */

#if LOG_PRINT_USE_UART0
  /*
   * Prepares the Serial driver 1.
   * Default is 38400-8-N-1. But here use uart0_config. 
   */
  sdStart(&SD1, &uart0_config);
#endif
}
