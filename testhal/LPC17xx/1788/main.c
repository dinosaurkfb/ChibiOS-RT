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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "test_hal.h"
#include "devices_lib.h"

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
/* EEPROM I2C interface config */
static const I2CConfig eeprom_i2ccfg = {
    I2C_STANDARD_MODE,
    400000
};

extern ROMCONST testcase_t * ROMCONST pattern_i2c[];
/*
 * Array of all the test patterns.
 */
static ROMCONST testcase_t * ROMCONST *patterns[] = {
  pattern_i2c,
  NULL
};

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/


/*===========================================================================*/
/* Test related code.                                                        */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
  msg_t result;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  EEPROMInit(&I2CD2, &eeprom_i2ccfg);

  LOG_PRINT("patterns :0x%08x\n", patterns);
  LOG_PRINT("&patterns[1] :0x%08x\n", &patterns[1]);
  LOG_PRINT("patterns[1] :0x%08x\n", patterns[1]);
  LOG_PRINT("pattern_i2c :0x%08x\n", pattern_i2c);
  LOG_PRINT("&pattern_i2c[1] :0x%08x -- 0x%08x\n", &pattern_i2c[1], pattern_i2c[1]);
  LOG_PRINT("&pattern_i2c[2] :0x%08x -- 0x%08x\n", &pattern_i2c[2], pattern_i2c[2]);
  LOG_PRINT("&pattern_i2c[3] :0x%08x -- 0x%08x\n", &pattern_i2c[3], pattern_i2c[3]);
  LOG_PRINT("&patterns[0][1] :0x%08x\n", &patterns[0][1]);
  LOG_PRINT("patterns[0][1] :0x%08x\n", patterns[0][1]);
  LOG_PRINT("patterns[0][0] :0x%08x\n", patterns[0][0]);
  LOG_PRINT("------------------------------\n");
  result = TestThread(&SD1, &patterns);
  LOG_PRINT("TestThread return %d\n", result);
  while(TRUE) {
    chThdSleepMilliseconds(100);
  }
}
