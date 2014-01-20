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
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __CLK_DIV(x,y) (((y) == 0) ? 0: (x)/(y))

#define XTAL        (12000000UL)        /* Oscillator frequency               */
#define OSC_CLK     (      XTAL)        /* Main oscillator frequency          */
#define RTC_CLK     (   32768UL)        /* RTC oscillator frequency           */
#define IRC_OSC     (12000000UL)        /* Internal RC oscillator frequency   */
#define WDT_OSC		  (  500000UL)		/* Internal WDT oscillator frequency  */


#define SCS_Val               0x00000021
#define CLKSRCSEL_Val         0x00000001
#define PLL0_SETUP            1
#define PLL0CFG_Val           0x00000009
#define PLL1_SETUP            1
#define PLL1CFG_Val           0x00000023
#define CCLKSEL_Val           0x00000101
#define USBCLKSEL_Val         0x00000201
#define EMCCLKSEL_Val         0x00000001
#define PCLKSEL_Val           0x00000002
#define PCONP_Val             0x042887DE
#define CLKOUTCFG_Val         0x00000100


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

/*----------------------------------------------------------------------------
  Check the register settings
  *----------------------------------------------------------------------------*/
#define CHECK_RANGE(val, min, max)                ((val < min) || (val > max))
#define CHECK_RSVD(val, mask)                     (val & mask)

/* Clock Configuration -------------------------------------------------------*/
#if (CHECK_RSVD((SCS_Val),       ~0x0000003F))
#error "SCS: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((CLKSRCSEL_Val), 0, 1))
#error "CLKSRCSEL: Value out of range!"
#endif

#if (CHECK_RSVD((PLL0CFG_Val),   ~0x0000007F))
#error "PLL0CFG: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PLL1CFG_Val),   ~0x0000007F))
#error "PLL1CFG: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((CCLKSEL_Val),   ~0x0000011F))
#error "CCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((USBCLKSEL_Val), ~0x0000031F))
#error "USBCLKCFG: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((EMCCLKSEL_Val), ~0x00000001))
#error "EMCCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PCLKSEL_Val), ~0x0000001F))
#error "PCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((PCONP_Val), ~0xFFFEFFFF))
#error "PCONP: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((CLKOUTCFG_Val), ~0x000001FF))
#error "CLKOUTCFG: Invalid values of reserved bits!"
#endif

/* Flash Accelerator Configuration -------------------------------------------*/
#if (CHECK_RSVD((FLASHCFG_Val), ~0x0000F000))
#warning "FLASHCFG: Invalid values of reserved bits!"
#endif


/*----------------------------------------------------------------------------
  DEFINES
  *----------------------------------------------------------------------------*/
/* pll_out_clk = F_cco / (2 × P)
   F_cco = pll_in_clk × M × 2 × P */
#define __M                   ((PLL0CFG_Val & 0x1F) + 1)
#define __PLL0_CLK(__F_IN)    (__F_IN * __M)
#define __CCLK_DIV            (CCLKSEL_Val & 0x1F)
#define __PCLK_DIV     (PCLKSEL_Val & 0x1F)
#define __ECLK_DIV     ((EMCCLKSEL_Val & 0x01) + 1)

/* Determine core clock frequency according to settings */

#if ((CLKSRCSEL_Val & 0x01) == 1) && ((SCS_Val & 0x20)== 0)
#error "Main Oscillator is selected as clock source but is not enabled!"
#endif

#if ((CCLKSEL_Val & 0x100) == 0x100) && (PLL0_SETUP == 0)
#error "Main PLL is selected as clock source but is not enabled!"
#endif

#if ((CCLKSEL_Val & 0x100) == 0)      /* cclk = sysclk */
#if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
#define __CORE_CLK (IRC_OSC / __CCLK_DIV)
#define __PER_CLK  (IRC_OSC/  __PCLK_DIV)
#define __EMC_CLK  (__CORE_CLK/  __ECLK_DIV)
#else                               /* sysclk = osc_clk */
#define __CORE_CLK (OSC_CLK / __CCLK_DIV)
#define __PER_CLK  (OSC_CLK/  __PCLK_DIV)
#define __EMC_CLK  (__CORE_CLK/  __ECLK_DIV)
#endif
#else                                 /* cclk = pll_clk */
#if ((CLKSRCSEL_Val & 0x01) == 0)   /* sysclk = irc_clk */
#define __CORE_CLK (__PLL0_CLK(IRC_OSC) / __CCLK_DIV)
#define __PER_CLK  (__PLL0_CLK(IRC_OSC) / __PCLK_DIV)
#define __EMC_CLK  (__CORE_CLK / __ECLK_DIV)
#else                               /* sysclk = osc_clk */
#define __CORE_CLK (__PLL0_CLK(OSC_CLK) / __CCLK_DIV)
#define __PER_CLK  (__PLL0_CLK(OSC_CLK) / __PCLK_DIV)
#define __EMC_CLK  (__CORE_CLK / __ECLK_DIV)
#endif
#endif

/** @addtogroup LPC177x_8x_System_Public_Variables  LPC177x_8x System Public Variables
    @{
*/
/*----------------------------------------------------------------------------
  Clock Variable definitions
  *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __CORE_CLK;/*!< System Clock Frequency (Core Clock)*/
uint32_t PeripheralClock = __PER_CLK; /*!< Peripheral Clock Frequency (Pclk)  */
uint32_t EMCClock   = __EMC_CLK; /*!< EMC Clock Frequency       */
uint32_t USBClock    = (48000000UL);    /*!< USB Clock Frequency - this value will
					  be updated after call SystemCoreClockUpdate, should be 48MHz*/
/**
 * @}
 */

/** @addtogroup LPC177x_8x_System_Public_Functions  LPC177x_8x System Public Functions
    @{
*/
/*----------------------------------------------------------------------------
  Clock functions
  *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* Determine clock frequency according to clock register values             */
  if ((LPC_SC->CCLKCFG &0x100) == 0) {            /* cclk = sysclk    */
    if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
      SystemCoreClock = __CLK_DIV(IRC_OSC , (LPC_SC->CCLKCFG & 0x1F));
      PeripheralClock = __CLK_DIV(IRC_OSC , (LPC_SC->PCLKSEL & 0x1F));
      EMCClock        = (SystemCoreClock / ((LPC_SC->EMCCLKSEL & 0x01)+1));
    }
    else {                                        /* sysclk = osc_clk */
      if ((LPC_SC->SCS & 0x40) == 0) {
	SystemCoreClock = 0;                      /* this should never happen! */
	PeripheralClock = 0;
	EMCClock        = 0;
      }
      else {
	SystemCoreClock = __CLK_DIV(OSC_CLK , (LPC_SC->CCLKCFG & 0x1F));
	PeripheralClock = __CLK_DIV(OSC_CLK , (LPC_SC->PCLKSEL & 0x1F));    
	EMCClock        = (SystemCoreClock / ((LPC_SC->EMCCLKSEL & 0x01)+1));
      }
    }
  }
  else {                                          /* cclk = pll_clk */
    if ((LPC_SC->PLL0STAT & 0x100) == 0) {        /* PLL0 not enabled */
      SystemCoreClock = 0;                      /* this should never happen! */
      PeripheralClock = 0;
      EMCClock     = 0;
    }
    else {
      if ((LPC_SC->CLKSRCSEL & 0x01) == 0) {    /* sysclk = irc_clk */
	uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
	uint8_t cpu_div = (LPC_SC->CCLKCFG & 0x1F);
	uint8_t per_div = (LPC_SC->PCLKSEL & 0x1F);
	uint8_t emc_div = (LPC_SC->EMCCLKSEL & 0x01)+1;
	SystemCoreClock = __CLK_DIV(IRC_OSC * mul , cpu_div);
	PeripheralClock = __CLK_DIV(IRC_OSC * mul , per_div);
	EMCClock        = SystemCoreClock / emc_div;
      }
      else {                                        /* sysclk = osc_clk */
        if ((LPC_SC->SCS & 0x40) == 0) {
          SystemCoreClock = 0;                      /* this should never happen! */
          PeripheralClock = 0;
          EMCClock     = 0;
        }
        else {
          uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
          uint8_t cpu_div = (LPC_SC->CCLKCFG & 0x1F);
          uint8_t per_div = (LPC_SC->PCLKSEL & 0x1F);
	  uint8_t emc_div = (LPC_SC->EMCCLKSEL & 0x01)+1;
          SystemCoreClock = __CLK_DIV(OSC_CLK * mul , cpu_div);
          PeripheralClock = __CLK_DIV(OSC_CLK * mul , per_div);
          EMCClock        = SystemCoreClock / emc_div;
        }
      }
    }
  }
  /* ---update USBClock------------------*/
  if(LPC_SC->USBCLKCFG & (0x01<<8))//Use PLL0 as the input to the USB clock divider
    {
      switch (LPC_SC->USBCLKCFG & 0x1F)
	{
	case 0:
	  USBClock = 0; //no clock will be provided to the USB subsystem
	  break;
	case 4:
	case 6:
	  {
	    uint8_t mul = ((LPC_SC->PLL0STAT & 0x1F) + 1);
	    uint8_t usb_div = (LPC_SC->USBCLKCFG & 0x1F);
	    if(LPC_SC->CLKSRCSEL & 0x01) //pll_clk_in = main_osc
	      USBClock = OSC_CLK * mul / usb_div;
	    else //pll_clk_in = irc_clk
	      USBClock = IRC_OSC * mul / usb_div;
	  }
	  break;
	default:
	  USBClock = 0;  /* this should never happen! */
	}
    }
  else if(LPC_SC->USBCLKCFG & (0x02<<8))//usb_input_clk = alt_pll (pll1)
    {
      if(LPC_SC->CLKSRCSEL & 0x01) //pll1_clk_in = main_osc
	USBClock = (OSC_CLK * ((LPC_SC->PLL1STAT & 0x1F) + 1));
      else //pll1_clk_in = irc_clk
	USBClock = (IRC_OSC * ((LPC_SC->PLL0STAT & 0x1F) + 1));
    }
  else
    USBClock = 0; /* this should never happen! */
}

/* Determine clock frequency according to clock register values             */

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

  LPC_SC->SCS       = SCS_Val;
  if (SCS_Val & (1 << 5)) {             /* If Main Oscillator is enabled      */
    while ((LPC_SC->SCS & (1<<6)) == 0);/* Wait for Oscillator to be ready    */
  }

  LPC_SC->CLKSRCSEL = CLKSRCSEL_Val;    /* Select Clock Source for sysclk/PLL0*/

#if (PLL0_SETUP)
  LPC_SC->PLL0CFG   = PLL0CFG_Val;
  LPC_SC->PLL0CON   = 0x01;             /* PLL0 Enable                        */
  LPC_SC->PLL0FEED  = 0xAA;
  LPC_SC->PLL0FEED  = 0x55;
  while (!(LPC_SC->PLL0STAT & (1<<10)));/* Wait for PLOCK0                    */
#endif

#if (PLL1_SETUP)
  LPC_SC->PLL1CFG   = PLL1CFG_Val;
  LPC_SC->PLL1CON   = 0x01;             /* PLL1 Enable                        */
  LPC_SC->PLL1FEED  = 0xAA;
  LPC_SC->PLL1FEED  = 0x55;
  while (!(LPC_SC->PLL1STAT & (1<<10)));/* Wait for PLOCK1                    */
#endif

  LPC_SC->CCLKCFG   = CCLKSEL_Val;      /* Setup Clock Divider                */
  LPC_SC->USBCLKCFG = USBCLKSEL_Val;    /* Setup USB Clock Divider            */
  LPC_SC->EMCCLKSEL = EMCCLKSEL_Val;    /* EMC Clock Selection                */
  LPC_SC->PCLKSEL   = PCLKSEL_Val;      /* Peripheral Clock Selection         */
  LPC_SC->PCONP     = PCONP_Val;        /* Power Control for Peripherals      */
  LPC_SC->CLKOUTCFG = CLKOUTCFG_Val;    /* Clock Output Configuration         */

  LPC_SC->PBOOST  |= 0x03;   /* Power Boost control    */

#if (FLASH_SETUP == 1)                  /* Flash Accelerator Setup            */
  LPC_SC->FLASHCFG  = FLASHCFG_Val|0x03A;
#endif
#ifdef  __RAM_MODE__
  SCB->VTOR  = 0x10000000 & 0x3FFFFF80;
#else
  SCB->VTOR  = 0x00000000 & 0x3FFFFF80;
#endif
  SystemCoreClockUpdate(); 
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
