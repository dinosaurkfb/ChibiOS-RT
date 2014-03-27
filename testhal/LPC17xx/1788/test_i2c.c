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
#include "lpc_types.h"

#include "string.h"
#include "devices_lib.h"

/* buffers depth */
#define TEST_BYTES EEPROM_SIZE

static uint8_t rxbuf[EEPROM_SIZE];
static uint8_t txbuf[EEPROM_SIZE];

/* EEPROM I2C interface config */
static const I2CConfig eeprom_i2ccfg = {
    I2C_STANDARD_MODE,
    400000
};

/* at24c0x_write_byte test */
static void write_byte_setup(void) {
  uint32_t i = 0;
  for (i = 0; i < EEPROM_SIZE; i++) {
    txbuf[i] = i;
  }
}

static void write_byte_exe(void) {
  uint32_t i = 0;
  uint32_t addr = 0;
  uint32_t test_bytes = TEST_BYTES;

  for (i = 0; i < MIN(test_bytes, EEPROM_SIZE); i++, addr++) {
    test_assert(i, RDY_OK == at24c0x_write_byte(&I2CD2, addr, txbuf[i]), "write byte failed.");
  }
}

ROMCONST testcase_t write_byte = {
  "write byte",
  write_byte_setup,
  NULL,
  write_byte_exe
};


/* at24c0x_random_read test */
static void random_read_setup(void) {
  uint32_t i = 0;
  for (i = 0; i < EEPROM_SIZE; i++) {
    rxbuf[i] = 0;
  }
}

static void random_read_exe(void) {
  uint32_t i = 0;
  uint32_t addr = 0;
  uint32_t test_bytes = TEST_BYTES;
  for (i = 0; i < MIN(test_bytes, EEPROM_SIZE); i++, addr++) {
    test_assert(i, RDY_OK == at24c0x_random_read(&I2CD2, addr, &rxbuf[i]), "random_read failed.");
    /* if (i == 0) { */
    /*   I2C_Dump(); */
    /* } */
  }
}

ROMCONST testcase_t random_read = {
  "random read byte",
  random_read_setup,
  NULL,
  random_read_exe
};


/* test consistency between read data and written data  */
static void write_read_cmp_setup(void) {
}

static void write_read_cmp_exe(void) {
  uint32_t i = 0;
  uint32_t test_bytes = TEST_BYTES;
  for (i = 0; i < MIN(test_bytes, EEPROM_SIZE); i++) {
    test_assert(i, rxbuf[i] == txbuf[i], "write read compare failed.");
  }
}

ROMCONST testcase_t write_read_cmp = {
  "write read compare",
  write_read_cmp_setup,
  NULL,
  write_read_cmp_exe
};


/* at24c0x_cur_read test */
static void cur_read_setup(void) {
  uint32_t i = 0;
  for (i = 0; i < EEPROM_SIZE; i++) {
    rxbuf[i] = 0;
  }
}

static void cur_read_exe(void) {
  uint32_t i = 0;
  uint32_t addr = 0;
  uint32_t test_bytes = TEST_BYTES;
  uint8_t rbyte = 0;
  /* Read the last byte of EEPROM to set the next internal pointer to EEPROM start */
  at24c0x_random_read(&I2CD2, EEPROM_SIZE - 1, &rbyte);

  for (i = 0; i < MIN(test_bytes, EEPROM_SIZE); i++, addr++) {
    test_assert(i, RDY_OK == at24c0x_cur_read(&I2CD2, &rxbuf[i]), "current_read failed.");
  }
}

ROMCONST testcase_t cur_read = {
  "cur_read byte",
  cur_read_setup,
  NULL,
  cur_read_exe
};

static uint32_t points[] = {0, 1, 2, 3, 5, 8, 9, 16, 23, EEPROM_SIZE/2, EEPROM_SIZE};

/* WriteEEPROM test */
static void write_eeprom_setup(void) {
}

static void write_eeprom_exe(void) {
  uint32_t i = 0;
  uint32_t addr = 0;
  for (i = 0; i < sizeof(points)/sizeof(points[0]); i++) {
    test_assert(i, RDY_OK == WriteEEPROM(addr, txbuf, points[i]), "write_eeprom failed.");
  }
}

ROMCONST testcase_t write_eeprom = {
  "write_eeprom",
  write_eeprom_setup,
  NULL,
  write_eeprom_exe
};

/* ReadEEPROM test */
static void read_eeprom_setup(void) {
  uint32_t i = 0;
  for (i = 0; i < EEPROM_SIZE; i++) {
    rxbuf[i] = 0;
  }
}

static void read_eeprom_exe(void) {
  uint32_t i = 0;
  uint32_t addr = 0;
  for (i = 0; i < sizeof(points)/sizeof(points[0]); i++) {
    test_assert(i, RDY_OK == ReadEEPROM(addr, rxbuf, points[i]), "read_eeprom failed.");
  }
}

ROMCONST testcase_t read_eeprom = {
  "read_eeprom",
  read_eeprom_setup,
  NULL,
  read_eeprom_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_i2c[] = {
  /* &write_byte, */
  /* &random_read, */
  /* &write_read_cmp, */
  /* &cur_read, */
  /* &write_read_cmp, */
  &write_eeprom,
  &read_eeprom,
  &write_read_cmp,
  NULL
};
