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

#include "update.h"
#include "at24c0x.h"
/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define NO_TEST  TRUE

#if ENABLE_IAP
static WORKING_AREA(waUpdaterThread, 128);
static msg_t UpdaterThread(void *arg) {
  (void)arg;
  LOG_PRINT("*** Thread updater.\n");
  chRegSetThreadName("updater");

  /* Work loop.*/
  while (TRUE) {
    /* Waiting for a update packet.*/
    uart0_scan();
  }
  return RDY_OK;
}
#endif /* #if ENABLE_IAP */

/* buffers depth */
#define RX_DEPTH 256
#define TX_DEPTH 256

static uint8_t rxbuf[RX_DEPTH];
static uint8_t txbuf[TX_DEPTH];

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
  
#if ENABLE_IAP
  chThdCreateStatic(waUpdaterThread, sizeof waUpdaterThread,
		    NORMALPRIO - 20, UpdaterThread, NULL);
  chThdSleepMilliseconds(50);
#endif /* #if ENABLE_IAP */

  int32_t w_ret = -1;
  int32_t r_ret = -1;
  EEPROMInit(&I2CD2);
 
  LOG_PRINT("*** Test at24c0x_write_byte\n");
  uint8_t byte = 57;
  uint8_t addr = 0x01;
  status = at24c0x_write_byte(&I2CD2, addr, byte);
  LOG_PRINT("write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  status = at24c0x_write_byte(&I2CD2, addr, byte);
  LOG_PRINT("write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  status = at24c0x_write_byte(&I2CD2, addr, byte);
  LOG_PRINT("write %d to 0x%02x, return: %d\n", byte++, addr++, status);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_write_byte end.\n\n");

  LOG_PRINT("*** Test at24c0x_random_read\n");
  uint8_t rbyte = 0;
  addr = 0x01;
  status = at24c0x_random_read(&I2CD2, addr, &rbyte);
  LOG_PRINT("random_read from 0x%02x, return: %d\n", addr, status);
  LOG_PRINT("read rbyte: %d\n", rbyte);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_random_read end.\n\n");

  LOG_PRINT("*** Test at24c0x_cur_read\n");
  status = at24c0x_cur_read(&I2CD2, &rbyte);
  LOG_PRINT("cur_read, return: %d\n", status);
  LOG_PRINT("read rbyte: %d\n", rbyte);
  status = at24c0x_cur_read(&I2CD2, &rbyte);
  LOG_PRINT("cur_read, return: %d\n", status);
  LOG_PRINT("read rbyte: %d\n", rbyte);
  /* I2C_Dump(); */
  LOG_PRINT("*** Test at24c0x_cur_read end.\n\n");


  LOG_PRINT("*** Test WriteEEPROM and ReadEEPROM\n");
  int32_t i = 0;
  for (i = 0; i < TX_DEPTH; i++) {
    txbuf[i] = i;
  }

  int32_t readbytes = 120;
  w_ret = WriteEEPROM(0, txbuf, readbytes);
  LOG_PRINT("WriteEEPROM write %d bytes, return: %d\n", readbytes, w_ret);
  I2C_Dump();
  if (-1 == w_ret) {
    LOG_PRINT("WriteEEPROM test failed\n");
    goto loop;
  }
  
  r_ret = ReadEEPROM(0, rxbuf, readbytes);
  LOG_PRINT("ReadEEPROM read %d bytes, return: %d\n", readbytes, r_ret);
  //  I2C_Dump();
  if (-1 == r_ret) {
    LOG_PRINT("ReadEEPROM test failed\n");
    goto loop;
  }

  if (0 == w_ret && 0 == r_ret) {
    if (!memcmp(rxbuf, txbuf, readbytes)) {
      LOG_PRINT("WriteEEPROM and ReadEEPROM test ok\n\n");
    } else {
      LOG_PRINT("WriteEEPROM and ReadEEPROM test failed\n");
      print_buf(rxbuf, readbytes);
    }
  }

 loop:
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
