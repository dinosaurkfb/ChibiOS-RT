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

#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "test_hal.h"

#include "update.h"

static bool cb1_called = FALSE;

/* Triggered when button is released. LED5 is set to ON or OFF alternately.*/
static void ext1_t1cb(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  cb1_called = TRUE;
  ToggleLED(5);
}

/*
 * EXT1_T1 configuration.
 */
static const EXTConfig ext1_t1cfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, ext1_t1cb},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static void ext1_t1_setup(void) {
  extStart(&EXTD1, &ext1_t1cfg);
}

static void ext1_t1_td(void) {
  extStop(&EXTD1);
}

#define WAIT_SECONDS       8

#define SIGNAL_LED         11

/* EXT1_T1 extPolledDelay test */
static void ext1_t1_exe(void) {
  /*
   * Call extPolledDelay to delay 'j' ms, and wait for the call return,
   * and then examine the time this call used.
   */
  LOG_PRINT("While LED%d is ON, LED5 will toggle along with KEY2 pressing.\n"
            "While LED%d is OFF, pressing KEY2 has no effect.\n"
            "Make sure to press at least once while LED%d is ON "
            "to pass the test or test will fail.\n",
            SIGNAL_LED,
            SIGNAL_LED,
            SIGNAL_LED);
  chThdSleepMilliseconds(S2ST(5));
  LOG_PRINT("5\n");
  chThdSleepMilliseconds(S2ST(1));
  LOG_PRINT("4\n");
  chThdSleepMilliseconds(S2ST(1));
  LOG_PRINT("3\n");
  chThdSleepMilliseconds(S2ST(1));
  LOG_PRINT("2\n");
  chThdSleepMilliseconds(S2ST(1));
  LOG_PRINT("1\n");
  chThdSleepMilliseconds(S2ST(1));
  LOG_PRINT("\n\n");

  LEDON(SIGNAL_LED);
  LOG_PRINT("LED ON, please press KEY2, this test will take %d seconds.",
            WAIT_SECONDS);
  LOG_PRINT("-----------------------------------------------------------\n\n\n");
  extChannelEnable(&EXTD1, 2);
  chThdSleepMilliseconds(S2ST(WAIT_SECONDS));
  test_assert(1, cb1_called == TRUE, "KEY2 not pressed.");
  extChannelDisable(&EXTD1, 2);
  cb1_called = FALSE;
  LEDOFF(SIGNAL_LED);
  LOG_PRINT("LED OFF, KEY2 ext channel is disabled for %d seconds, during"
            " this period, pressing KEY2 has no effect.\n",
            WAIT_SECONDS);
  LOG_PRINT("-----------------------------------------------------------\n\n\n");
  chThdSleepMilliseconds(S2ST(WAIT_SECONDS));
  LEDON(SIGNAL_LED);
  LOG_PRINT("LED ON, KEY2 ext channel is enabled again.\n"
            "Press KEY2 within %d seconds to pass the test or not to fail"
            " the test.\nLED5 will toggle along with your every pressing.\n",
            WAIT_SECONDS);
  LOG_PRINT("-----------------------------------------------------------\n\n\n");
  extChannelEnable(&EXTD1, 2);
  chThdSleepMilliseconds(S2ST(WAIT_SECONDS));
  test_assert(2, cb1_called == TRUE, "KEY2 not pressed.");
}

ROMCONST testcase_t ext1_t1 = {
  "ext1_t1",
  ext1_t1_setup,
  ext1_t1_td,
  ext1_t1_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_ext[] = {
  &ext1_t1,
  NULL
};
