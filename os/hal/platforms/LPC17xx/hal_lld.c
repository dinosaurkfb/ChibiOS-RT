/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio
    LPC17xx HAL driver - Copyright (C) 2013 Marcin Jokel

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

/**
 * @file    LPC17xx/hal_lld.c
 * @brief   LPC17xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/**
 * @brief   Register missing in NXP header file.
 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#ifdef LPC177x_8x

#define USBCLKSEL_Val      0x00000201
#define EMCCLKSEL_Val      0x00000001
#define PCONP_Val          0x042887DE
#define CLKOUTCFG_Val      0x00000100


/*--------------------- Flash Accelerator Configuration ----------------------
//
// <e> Flash Accelerator Configuration register (FLASHCFG - address 0x400F C000)
//  <o1.12..15> FLASHTIM: Flash Access Time
//     <0=> 1 CPU clock (for CPU clock up to 20 MHz)
//     <1=> 2 CPU clocks (for CPU clock up to 40 MHz)
//     <2=> 3 CPU clocks (for CPU clock up to 60 MHz)
//     <3=> 4 CPU clocks (for CPU clock up to 80 MHz)
//     <4=> 5 CPU clocks (for CPU clock up to 100 MHz)
//     <5=> 6 CPU clocks (for any CPU clock)
// </e>
*/

#define FLASH_SETUP           1
#define FLASHCFG_Val          0x00005000

/*
//-------- <<< end of configuration section >>> ------------------------------
*/

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void LPC177x_8x_clock_init(void) {

#if LPC17xx_MAINOSC_ENABLE
  LPC_SC->SCS = (1 << 5) | (LPC17xx_OSCRANGE << 4);     /* Enable Main oscillator */
  while (!(LPC_SC->SCS & (1 << 6)))
    ;                                                   /* Wait for main oscillator to be ready */
#endif

  LPC_SC->CLKSRCSEL = LPC17xx_SYSCLK_SELECT;    /* Select Clock Source for sysclk/PLL0*/

#if LPC17xx_MAINOSC_ENABLE
  /* PLL0 configuration and start */
  LPC_SC->PLL0CFG = (LPC17xx_PLL0CFG_PSEL0 << 5) | LPC17xx_PLL0CFG_MSEL0;

  LPC_SC->PLL0CON   = 0x01;             /* PLL0 Enable                        */
  LPC_SC->PLL0FEED  = 0xAA;
  LPC_SC->PLL0FEED  = 0x55;
  while (!(LPC_SC->PLL0STAT & (1<<10)));/* Wait for PLOCK0                    */
#endif

#if (LPC17xx_ALTPLL_ENABLE)
  LPC_SC->PLL1CFG   = (LPC17xx_PLL1CFG_PSEL1 << 5) | LPC17xx_PLL1CFG_MSEL1;
  LPC_SC->PLL1CON   = 0x01;             /* PLL1 Enable                        */
  LPC_SC->PLL1FEED  = 0xAA;
  LPC_SC->PLL1FEED  = 0x55;
  while (!(LPC_SC->PLL1STAT & (1<<10)));/* Wait for PLOCK1                    */
#endif

  LPC_SC->CCLKCFG   = (1UL << 8) | LPC17xx_CCLK_DIV; /* Setup CPU Clock Divider */
  LPC_SC->USBCLKCFG = USBCLKSEL_Val;    /* Setup USB Clock Divider            */
  LPC_SC->EMCCLKSEL = EMCCLKSEL_Val;    /* EMC Clock Selection                */
  LPC_SC->PCLKSEL   = LPC17xx_PCLKSEL;  /* Peripheral Clock Selection         */
  LPC_SC->PCONP     = PCONP_Val;        /* Power Control for Peripherals      */
  LPC_SC->CLKOUTCFG = 0x00000100;    /* Clock Output Configuration         */

  LPC_SC->PBOOST  |= 0x03;   /* Power Boost control    */

#if (FLASH_SETUP == 1)                  /* Flash Accelerator Setup            */
  LPC_SC->FLASHCFG  = FLASHCFG_Val|0x03A;
#endif
#ifdef  __RAM_MODE__
  SCB->VTOR  = 0x10000000 & 0x3FFFFF80;
#else
  SCB->VTOR  = 0x00000000 & 0x3FFFFF80;
#endif
}
#endif //ifdef LPC177x_8x

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* SysTick initialization using the system clock.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, CORTEX_PRIORITY_SYSTICK);
  SysTick->LOAD = LPC17xx_CCLK / CH_FREQUENCY - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;
}

/**
 * @brief   LPC17xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void LPC17xx_clock_init(void) {
#ifdef LPC177x_8x
  LPC177x_8x_clock_init();
#else //ifdef LPC177x_8x
  /* Flash wait states setting, the code takes care to not touch TBD bits.*/
  LPC_SC->FLASHCFG = (LPC_SC->FLASHCFG & ~(0x0000000F << 12)) | (LPC17xx_FLASHCFG_FLASHTIM << 12);

  /* System oscillator initialization if required.*/

#if LPC17xx_MAINOSC_ENABLE
  LPC_SC->SCS = (1 << 5) | (LPC17xx_OSCRANGE << 4);     /* Enable Main oscillator */
  while (!(LPC_SC->SCS & (1 << 6)))
    ;                                                   /* Wait for main oscillator to be ready */
#endif

  /* Peripheral clock divider initialization, must be set before enabling Main PLL (PLL0).
     Read errata sheet ES_LPC176x. */
  LPC_SC->PCLKSEL0 = LPC17xx_PCLKSEL0;
  LPC_SC->PCLKSEL1 = LPC17xx_PCLKSEL1;

  LPC_SC->CCLKCFG = LPC17xx_CCLK_DIV - 1;               /* Set CPU clock divider */

  LPC_SC->CLKSRCSEL = LPC17xx_SYSCLK_SELECT;            /* Select clock source for PLL0 if enabled or CPU */

#if LPC17xx_MAINPLL_ENABLE

  /* PLL0 configuration and start */
  LPC_SC->PLL0CFG = (LPC17xx_PLL0CFG_NSEL0 << 16) | LPC17xx_PLL0CFG_MSEL0;
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;

  LPC_SC->PLL0CON = 0x01;      	/* Enable PLL0. */
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;
  while (!(LPC_SC->PLL0STAT & (1UL << 26)))
      ;                         /* Wait for PLL0 locked */

  LPC_SC->PLL0CON = 0x03;      	/* Enable and Connect PLL0. */
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;
  while (!(LPC_SC->PLL0STAT & ((1UL << 25) | (1UL << 24))))
        ;                       /* Wait for PLL0 connected */

#endif /* LPC17xx_MAINPLL_ENABLE == TRUE */

#if LPC17xx_USBPLL_ENABLE
  /* PLL1 configuration and start */
  LPC_SC->PLL1CFG = (LPC17xx_PLL1CFG_PSEL1 << 5) | LPC17xx_PLL1CFG_MSEL1;
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;

  LPC_SC->PLL1CON = 0x01;      	/* Enable PLL1. */
  LPC_SC->PLL1FEED = 0xAA;
  LPC_SC->PLL1FEED = 0x55;
  while (!(LPC_SC->PLL1STAT & (1UL << 10)))
    ;                           /* Wait for PLL1 locked */

   LPC_SC->PLL1CON = 0x03;      /* Enable and Connect PLL1. */
   LPC_SC->PLL1FEED = 0xAA;
   LPC_SC->PLL1FEED = 0x55;
   while (!(LPC_SC->PLL1STAT & ((1UL << 9) | (1UL << 8))))
     ;                          /* Wait for PLL1 connected */
#endif /* LPC17xx_USBPLL_ENABLE == TRUE */

#if !LPC17xx_USBPLL_ENABLE && HAL_USE_USB
   LPC_SC->USBCLKCFG = LPC17xx_USBCLKPLL0_SELECT;
#endif

  /* Power control configuration */
  LPC_SC->PCONP = (1 << 15) | (1 << 9); /* Enable power for GPIO and RTC */

#if LPC17xx_CLKOUT_ENABLE
  LPC_SC->CLKOUTCFG = (1UL << 8) | ((LPC17xx_CLKOUT_DIV - 1) << 4) | LPC17xx_CLKOUT_SELECT;
#endif

#endif //ifdef LPC177x_8x
}

/** @} */
