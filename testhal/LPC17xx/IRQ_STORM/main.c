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

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/

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

  /*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT IRQ-STORM long duration test\n");
  LOG_PRINT("***\n");
  LOG_PRINT("*** Kernel:       %u\n", CH_KERNEL_VERSION);
#ifdef CH_COMPILER_NAME
  LOG_PRINT("*** Compiler:     %u\n", CH_COMPILER_NAME);
#endif
  LOG_PRINT("*** Architecture: %u\n", CH_ARCHITECTURE_NAME);
#ifdef CH_CORE_VARIANT_NAME
  LOG_PRINT("*** Core Variant: %u\n", CH_CORE_VARIANT_NAME);
#endif
#ifdef CH_PORT_INFO
  LOG_PRINT("*** Port Info:    %u\n", CH_PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  LOG_PRINT("*** Platform:     %u\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  LOG_PRINT("*** Test Board:   %u\n", BOARD_NAME);
#endif
  LOG_PRINT("***\n");
  LOG_PRINT("*** System Clock: %u\n", LPC17xx_CCLK);
  LOG_PRINT("Test Complete\n");
  /*
   * Normal main() thread activity, nothing in this test.
   */
  uint32_t s = 0;
  while (TRUE) {
    chThdSleepMilliseconds(1000);
    LOG_PRINT("*** Alive %u seconds.\n", ++s);
  }
  return 0;
}
