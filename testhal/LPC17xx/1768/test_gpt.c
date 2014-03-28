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

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define GPT1_FREQ   1000000  /* 1MHz timer clock.*/

/* test repeat times for continuous mode */
#define MAX_CONTINUOUS_TIMES 10

/* maximum interval in miliseconds */
#define MAX_INTERVAL  2000

#define S2CNT(x)       (GPT1_FREQ * (x))
#define MS2CNT(x)      (GPT1_FREQ * (x) / 1000)
#define US2CNT(x)      (GPT1_FREQ * (x) / 1000000)


static volatile systime_t s_start_st = 0;
static volatile bool_t cb1_called = FALSE;

/*
 * GPT1_T1 callback.
 */
static void gpt1_t1cb(GPTDriver *gptp) {
  (void)gptp;
  chSysLockFromIsr();
  cb1_called = TRUE;
  chSysUnlockFromIsr();
}

/*
 * GPT1_T1 configuration.
 */
static const GPTConfig gpt1_t1cfg = {
  GPT1_FREQ,/* gpt1_t1 timer clock.*/
  gpt1_t1cb    /* Timer callback.*/
};

static void gpt1_t1_setup(void) {
  gptStart(&GPTD1, &gpt1_t1cfg);
}

static void gpt1_t1_td(void) {
  gptStop(&GPTD1);
}

/* GPT1_T1 gptStartOneShot test */
static void gpt1_t1_exe(void) {
  uint32_t i = 0;
  /*
   * Start timer with interval 'i', and wait for callback to be called,
   * and then examine this time interval.
   */
  for (i = 1; i < MAX_INTERVAL; i = i * (i+1)) {
    cb1_called = FALSE;
    gptStartOneShot(&GPTD1, MS2CNT(i));
    s_start_st = chTimeNow();
    while (!cb1_called);
    test_assert_time_window(i, s_start_st + MS2ST(i) - 1, s_start_st + MS2ST(i) + 1);
  }
}

ROMCONST testcase_t gpt1_t1 = {
  "gpt1_t1",
  gpt1_t1_setup,
  gpt1_t1_td,
  gpt1_t1_exe
};

/*
 * GPT1_T2 callback.
 */
static void gpt1_t2cb(GPTDriver *gptp) {
  (void)gptp;
  chSysLockFromIsr();
  cb1_called = TRUE;
  chSysUnlockFromIsr();
}

/*
 * GPT1_T2 configuration.
 */
static const GPTConfig gpt1_t2cfg = {
  GPT1_FREQ,/* gpt1_t2 timer clock.*/
  gpt1_t2cb    /* Timer callback.*/
};

static void gpt1_t2_setup(void) {
  gptStart(&GPTD1, &gpt1_t2cfg);
}

static void gpt1_t2_td(void) {
  gptStop(&GPTD1);
}

/* GPT1_T2 gptChangeInterval test */
static void gpt1_t2_exe(void) {
  uint32_t i = 0;
  uint32_t j = 0;
  gptStartContinuous(&GPTD1, MS2CNT(MAX_INTERVAL));

  /*
   * Change timer interval to 'j', and wait for callback to be called,
   * and then examine this time interval 'MAX_CONTINUOUS_TIMES' times.
   */
  for (j = 1; j < MAX_INTERVAL; j = j * (j+1)) {
    gptChangeInterval(&GPTD1, MS2CNT(j));
    for (i = 0; i < MAX_CONTINUOUS_TIMES; i++) {
      cb1_called = FALSE;
      s_start_st = chTimeNow();
      while (!cb1_called);
      test_assert_time_window(MAX_CONTINUOUS_TIMES * j + i,
                              s_start_st + MS2ST(j) - 1,
                              s_start_st + MS2ST(j) + 1);
    }
  }
  gptStop(&GPTD1);
}

ROMCONST testcase_t gpt1_t2 = {
  "gpt1_t2",
  gpt1_t2_setup,
  gpt1_t2_td,
  gpt1_t2_exe
};

/*
 * GPT1_T3 configuration.
 */
static const GPTConfig gpt1_t3cfg = {
  GPT1_FREQ,/* gpt1_t3 timer clock.*/
  gpt1_t2cb    /* Timer callback.*/
};

static void gpt1_t3_setup(void) {
  gptStart(&GPTD1, &gpt1_t3cfg);
}

static void gpt1_t3_td(void) {
  gptStop(&GPTD1);
}

/* GPT1_T3 gptStartContinuous and gptStopTimer test */
static void gpt1_t3_exe(void) {
  uint32_t i = 0;
  uint32_t j = 0;
  /*
   * Start timer with interval to 'j', and wait for callback to be called,
   * and then examine this time interval 'MAX_CONTINUOUS_TIMES' times, after
   * which, the timer is stoped and restarted in the next loop.
   */
  for (j = 1; j < MAX_INTERVAL; j = j * (j+1)) {
    gptStartContinuous(&GPTD1, MS2CNT(j));
    for (i = 0; i < MAX_CONTINUOUS_TIMES; i++) {
      cb1_called = FALSE;
      s_start_st = chTimeNow();
      while (!cb1_called);
      test_assert_time_window(MAX_CONTINUOUS_TIMES * j + i,
                              s_start_st + MS2ST(j) - 1,
                              s_start_st + MS2ST(j) + 1);
    }
    gptStopTimer(&GPTD1);
  }
}

ROMCONST testcase_t gpt1_t3 = {
  "gpt1_t3",
  gpt1_t3_setup,
  gpt1_t3_td,
  gpt1_t3_exe
};

/*
 * GPT1_T4 configuration.
 */
static const GPTConfig gpt1_t4cfg = {
  GPT1_FREQ,/* gpt1_t4 timer clock.*/
  gpt1_t2cb    /* Timer callback.*/
};

static void gpt1_t4_setup(void) {
  gptStart(&GPTD1, &gpt1_t4cfg);
}

static void gpt1_t4_td(void) {
  gptStop(&GPTD1);
}

/* GPT1_T4 gptPolledDelay test */
static void gpt1_t4_exe(void) {
  uint32_t j = 0;
  /*
   * Call gptPolledDelay to delay 'j' ms, and wait for the call return,
   * and then examine the time this call used.
   */
  for (j = 1; j < MAX_INTERVAL; j = j * (j+1)) {
    s_start_st = chTimeNow();
    gptPolledDelay(&GPTD1, MS2CNT(j));
    test_assert_time_window(j,
                            s_start_st + MS2ST(j) - 1,
                            s_start_st + MS2ST(j) + 1);
  }
}

ROMCONST testcase_t gpt1_t4 = {
  "gpt1_t4",
  gpt1_t4_setup,
  gpt1_t4_td,
  gpt1_t4_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_gpt[] = {
  &gpt1_t1,
  &gpt1_t2,
  &gpt1_t3,
  &gpt1_t4,
  NULL
};

