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

/**
 * @file    LPC17xx/ext_lld.c
 * @brief   LPC17xx EXT subsystem low level driver source.
 *
 * @addtogroup EXT
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_EXT || defined(__DOXYGEN__)

#include "ext_lld_isr.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EXTD1 driver identifier.
 */
EXTDriver EXTD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level EXT driver initialization.
 *
 * @notapi
 */
void ext_lld_init(void) {

  /* Driver initialization.*/
  extObjectInit(&EXTD1);
}

/**
 * @brief   Configures and activates the EXT peripheral.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_start(EXTDriver *extp) {
  int i;

  /* Configuration of automatic channels.*/
  for (i = 0; i < EXT_MAX_CHANNELS; i++)
    if (extp->config->channels[i].mode & EXT_CH_MODE_AUTOSTART) {
      ext_lld_channel_enable(extp, i);
    } else {
      ext_lld_channel_disable(extp, i);
    }
}

/**
 * @brief   Deactivates the EXT peripheral.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_stop(EXTDriver *extp) {
  int i;

  if (extp->state == EXT_ACTIVE)
    for (i = 0; i < EXT_MAX_CHANNELS; i++)
      ext_lld_exti_irq_disable(i);

  /* Clear interrupt flag */
  LPC_SC->EXTINT = EXT_CHANNELS_MASK;
}

/**
 * @brief   Enables an EXT channel.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be enabled
 *
 * @notapi
 */
void ext_lld_channel_enable(EXTDriver *extp, expchannel_t channel) {

  /* Select EINTx function for the pin */
  uint8_t offset = 20 + (2 * channel);
  LPC_PINCON->PINSEL4 = (LPC_PINCON->PINSEL4 &~ (0xFF << offset)) | (1 << offset);

  /* Set edge sensitive */
  LPC_SC->EXTMODE = 1 << channel;
  
  if (extp->config->channels[channel].mode & EXT_CH_MODE_RISING_EDGE)
    LPC_SC->EXTPOLAR |=  (1 << channel);
  else
    LPC_SC->EXTPOLAR &=  ~(1 << channel);

  /* Clear interrupt flag */
  LPC_SC->EXTINT |=  (1 << channel);

  ext_lld_exti_irq_enable( channel );
}

/**
 * @brief   Disables an EXT channel.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be disabled
 *
 * @notapi
 */
void ext_lld_channel_disable(EXTDriver *extp, expchannel_t channel) {
  (void)extp;

  ext_lld_exti_irq_disable(channel);

  /* Clear interrupt flag */
  LPC_SC->EXTINT |=  (1 << channel);
}

#endif /* HAL_USE_EXT */

/** @} */
