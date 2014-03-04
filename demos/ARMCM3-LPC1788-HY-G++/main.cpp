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

#include "ch.hpp"
#include "hal.h"


#include "update.h"

using namespace chibios_rt;
/*===========================================================================*/
/* Configurable settings.                                                    */
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
  System::init();

  /*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT\n");
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
  LOG_PRINT("*** Peripheral Clock: %u\n", LPC17xx_PCLK);
  /*
   * Normal main() thread activity, nothing in this test.
   */

  updateThreadStart();

  while (TRUE) {
    ToggleLED(1);
    ToggleLED(2);
    BaseThread::sleep(MS2ST(500));
  }
  return 0;
}
