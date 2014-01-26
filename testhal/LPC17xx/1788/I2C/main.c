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

#include "string.h"

#include "at24c0x.h"
/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define NO_TEST  TRUE

/* buffers depth */
#define RX_DEPTH 6
#define TX_DEPTH 6

static uint8_t rxbuf[RX_DEPTH];
static uint8_t txbuf[TX_DEPTH] = { 0x11, 0x12, 0x13, 0xAA, 0xCC, 0xFF};

void print_buf(uint8_t *buf, size_t len) {
  size_t i = 0;
  LOG_PRINT("\n***\n");
  for (; i < len ; i++) {
    LOG_PRINT("0x%x ", buf[i]);
  }
  LOG_PRINT("\n***\n");
}

/*
 * Application entry point.
 */
int main(void) {
  msg_t status;

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
#ifdef NO_TEST
  uint32_t s = 0;
#endif
  
  EEPROMInit(&I2CD1);
 
  LOG_PRINT("*** Test at24c0x_write_byte\n");
  uint8_t byte = 57;
  uint8_t addr = 0x01;
  status = at24c0x_write_byte(&I2CD1, addr, byte);
  LOG_PRINT("*** write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  status = at24c0x_write_byte(&I2CD1, addr, byte);
  LOG_PRINT("*** write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  status = at24c0x_write_byte(&I2CD1, addr, byte);
  LOG_PRINT("*** write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_write_byte end.\n\n");

  LOG_PRINT("*** Test at24c0x_random_read\n");
  uint8_t rbyte = 0;
  addr = 0x01;
  status = at24c0x_random_read(&I2CD1, addr, &rbyte);
  LOG_PRINT("*** random_read from 0x%02x, return: %d\n", addr, status);
  LOG_PRINT("*** read rbyte: %d\n", rbyte);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_random_read end.\n\n");

  LOG_PRINT("*** Test at24c0x_cur_read\n");
  status = at24c0x_cur_read(&I2CD1, &rbyte);
  LOG_PRINT("*** cur_read, return: %d\n", status);
  LOG_PRINT("*** read rbyte: %d\n", rbyte);
  status = at24c0x_cur_read(&I2CD1, &rbyte);
  LOG_PRINT("*** cur_read, return: %d\n", status);
  LOG_PRINT("*** read rbyte: %d\n", rbyte);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_cur_read end.\n\n");


  LOG_PRINT("*** Test WriteEEPROM and ReadEEPROM\n");
  status = WriteEEPROM(0, txbuf, TX_DEPTH);
  LOG_PRINT("*** WriteEEPROM return: %d\n", status);
  status = ReadEEPROM(0, rxbuf, RX_DEPTH);
  LOG_PRINT("*** ReadEEPROM return: %d\n", status);
  if (!memcmp(rxbuf, txbuf, RX_DEPTH)) {
    LOG_PRINT("*** WriteEEPROM and ReadEEPROM test ok\n\n");;
  }

  while (TRUE) {
    chThdSleepMilliseconds(1000);
#ifdef NO_TEST
    ++s;
    if (s % 30 == 0) {
      LOG_PRINT("*** Alive %u seconds.\n", s);
    }
#endif

  }
  return 0;
}
