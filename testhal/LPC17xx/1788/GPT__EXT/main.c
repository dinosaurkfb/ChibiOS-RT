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

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define TEST_GPT TRUE

#define NO_TEST  TRUE

#if (TEST_GPT == TRUE)
#undef NO_TEST

#ifndef RANDOMIZE
#define RANDOMIZE       FALSE
#endif

#ifndef ITERATIONS
#define ITERATIONS      100
#endif

#ifndef NUM_THREADS
#define NUM_THREADS     4
#endif

#ifndef MAILBOX_SIZE
#define MAILBOX_SIZE    4
#endif

#endif

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/


/*===========================================================================*/
/* Test related code.                                                        */
/*===========================================================================*/

#if (TEST_GPT == TRUE) 
#define MSG_SEND_LEFT   0
#define MSG_SEND_RIGHT  1

static bool_t saturated;

/*
 * Mailboxes and buffers.
 */
static Mailbox mb[NUM_THREADS];
static msg_t b[NUM_THREADS][MAILBOX_SIZE];

/*
 * Test worker threads.
 */
static WORKING_AREA(waWorkerThread[NUM_THREADS], 128);
static msg_t WorkerThread(void *arg) {
  static volatile unsigned x = 0;
  static unsigned cnt = 0;
  unsigned me = (unsigned)arg;
  unsigned target;
  unsigned r;
  msg_t msg;

  chRegSetThreadName("worker");

  /* Work loop.*/
  while (TRUE) {
    /* Waiting for a message.*/
    chMBFetch(&mb[me], &msg, TIME_INFINITE);

#if RANDOMIZE
    /* Pseudo-random delay.*/
    {
      chSysLock();
      r = rand() & 15;
      chSysUnlock();
      while (r--)
	x++;
    }
#else
    /* Fixed delay.*/
    {
      r = me >> 4;
      while (r--)
	x++;
    }
#endif

    /* Deciding in which direction to re-send the message.*/
    if (msg == MSG_SEND_LEFT)
      target = me - 1;
    else
      target = me + 1;

    if (target < NUM_THREADS) {
      /* If this thread is not at the end of a chain re-sending the message,
         note this check works because the variable target is unsigned.*/
      msg = chMBPost(&mb[target], msg, TIME_IMMEDIATE);
      if (msg != RDY_OK)
        saturated = TRUE;
    }
    else {
      /* Provides a visual feedback about the system.*/
      if (++cnt >= 500) {
        cnt = 0;
	ToggleLED(me+1);
      }
    }
  }
}

/*
 * GPT1 callback.
 */
static void gpt1cb(GPTDriver *gptp) {
  msg_t msg;

  (void)gptp;
  chSysLockFromIsr();
  msg = chMBPostI(&mb[0], MSG_SEND_RIGHT);
  if (msg != RDY_OK)
    saturated = TRUE;
  chSysUnlockFromIsr();
}

/*
 * GPT2 callback.
 */
static void gpt2cb(GPTDriver *gptp) {
  msg_t msg;

  (void)gptp;
  chSysLockFromIsr();
  msg = chMBPostI(&mb[NUM_THREADS - 1], MSG_SEND_LEFT);
  if (msg != RDY_OK)
    saturated = TRUE;
  chSysUnlockFromIsr();
}

/*
 * GPT1 configuration.
 */
static const GPTConfig gpt1cfg = {
  1000000,  /* 1MHz timer clock.*/
  gpt1cb    /* Timer callback.*/
};

/*
 * GPT2 configuration.
 */
static const GPTConfig gpt2cfg = {
  1000000,  /* 1MHz timer clock.*/
  gpt2cb    /* Timer callback.*/
};
#endif

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

#if (TEST_GPT == TRUE)
  uint32_t i;
  gptcnt_t interval, threshold, worst;

  gptStart(&GPTD1, &gpt1cfg);
  gptStart(&GPTD2, &gpt2cfg);

  /*
   * Initializes the mailboxes and creates the worker threads.
   */
  for (i = 0; i < NUM_THREADS; i++) {
    chMBInit(&mb[i], b[i], MAILBOX_SIZE);
    chThdCreateStatic(waWorkerThread[i], sizeof waWorkerThread[i],
                      NORMALPRIO - 20, WorkerThread, (void *)i);
  }
#endif

  /*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT IRQ-STORM long duration test\n");
  LOG_PRINT("***\n");
  LOG_PRINT("*** Kernel:       %s\n", CH_KERNEL_VERSION);
#ifdef CH_COMPILER_NAME
  LOG_PRINT("*** Compiler:     %s\n", CH_COMPILER_NAME);
#endif
  LOG_PRINT("*** Architecture: %s\n", CH_ARCHITECTURE_NAME);
#ifdef CH_CORE_VARIANT_NAME
  LOG_PRINT("*** Core Variant: %s\n", CH_CORE_VARIANT_NAME);
#endif
#ifdef CH_PORT_INFO
  LOG_PRINT("*** Port Info:    %s\n", CH_PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  LOG_PRINT("*** Platform:     %s\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  LOG_PRINT("*** Test Board:   %s\n", BOARD_NAME);
#endif
  LOG_PRINT("***\n");
  LOG_PRINT("*** System Clock: %u\n", LPC17xx_CCLK);
  /*
   * Normal main() thread activity, nothing in this test.
   */
#ifdef NO_TEST
  uint32_t s = 0;
#endif
#if (TEST_GPT == TRUE)
  LOG_PRINT("*** Iterations:   %u\n", ITERATIONS);
  LOG_PRINT("*** Randomize:    %u\n", RANDOMIZE);
  LOG_PRINT("*** Threads:      %u\n", NUM_THREADS);
  LOG_PRINT("*** Mailbox size: %u\n\n", MAILBOX_SIZE);
  worst = 0;
  for (i = 1; i <= ITERATIONS; i++){
    LOG_PRINT("Iteration %u\n", i);
    saturated = FALSE;
    threshold = 0;
    for (interval = 2000; interval >= 20; interval -= interval / 10) {
      gptStartContinuous(&GPTD1, interval - 1); /* Slightly out of phase.*/
      gptStartContinuous(&GPTD2, interval + 1); /* Slightly out of phase.*/
      chThdSleepMilliseconds(1000);
      gptStopTimer(&GPTD1);
      gptStopTimer(&GPTD2);
      if (!saturated)
        LOG_PRINT(".");
      else {
        LOG_PRINT("#");
        if (threshold == 0)
          threshold = interval;
      }
    }
    /* Gives the worker threads a chance to empty the mailboxes before next
       cycle.*/
    chThdSleepMilliseconds(20);
    LOG_PRINT("\n");
    LOG_PRINT("Saturated at %u uS\n\n", threshold);
    if (threshold > worst)
      worst = threshold;
  }
  gptStopTimer(&GPTD1);
  gptStopTimer(&GPTD2);

  LOG_PRINT("Worst case at %u uS\n", worst);
#endif

  while (TRUE) {
    chThdSleepMilliseconds(1000);
#ifdef NO_TEST
    ++s;
    LOG_PRINT("*** Alive %u seconds.\n", s);
    s % 2 == 0 ? LEDOFF(4) : LEDON(4);
#endif
  }
  return 0;
}
