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
 * @file    LPC17XX/can_lld.c
 * @brief   LPC17XX CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "pinsel_lld.h"
#include <string.h>

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CAN1 driver identifier.*/
#if LPC17XX_CAN_USE_CAN1 || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/** @brief CAN2 driver identifier.*/
#if LPC17XX_CAN_USE_CAN2 || defined(__DOXYGEN__)
CANDriver CAND2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Programs the filters.
 *
 * @param[in] can2sb    number of the first filter assigned to CAN2
 * @param[in] num       number of entries in the filters array, if zero then
 *                      a default filter is programmed
 * @param[in] cfp       pointer to the filters array, can be @p NULL if
 *                      (num == 0)
 *
 * @notapi
 */
static void can_lld_set_filters(uint32_t can2sb,
                         uint32_t num,
                         const CANFilter *cfp) {
	int i=can2sb;
	if(i>0) {
		i=num;
	}
	if(cfp ==NULL) {
		return;
	}
}

/**
 * @brief   Common TX ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_tx_handler(CANDriver *canp) {
  chSysLockFromIsr();
  while (chSemGetCounterI(&canp->txsem) < 0)
    chSemSignalI(&canp->txsem);
  chEvtBroadcastFlagsI(&canp->txempty_event, CAN_MAILBOX_TO_MASK(1));
  chSysUnlockFromIsr();
}

/**
 * @brief   Common CDO ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_cdo_handler(CANDriver *canp, uint32_t err_code) {
	canp->can->CMR = 0x08;
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&canp->error_event, err_code);
    chSysUnlockFromIsr();
}

/**
 * @brief   Common RX0 ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_rx_handler(CANDriver *canp) {
    chSysLockFromIsr();

	int index = (canp->rx_map.start + canp->rx_map.num) & LPC17XX_CAN_RX_BUF_MASK;
	CANRxFrame *crfp = canp->rx_map.buf + index;

    crfp->DLC = (uint8_t)(((canp->can->RFS) & 0x000F0000) >> 16); 
	crfp->RTR = (uint8_t)(((canp->can->RFS) & 0x40000000) >> 30); 
	crfp->IDE = (uint8_t)(((canp->can->RFS) & 0x80000000) >> 31); 
	if (crfp->IDE) 
		crfp->EID = canp->can->RID; 
	else 
		crfp->SID = canp->can->RID;

    crfp->data32[0] = canp->can->RDA;
    crfp->data32[1] = canp->can->RDB;

	if (canp->rx_map.num == LPC17XX_CAN_RX_BUF_NUM)
		canp->rx_map.start = (canp->rx_map.start+ 1) & LPC17XX_CAN_RX_BUF_MASK;
	else
		canp->rx_map.num++;


    /* release receive buffer */ 
    canp->can->CMR = 0x04;

    while (chSemGetCounterI(&canp->rxsem) < 0)
      chSemSignalI(&canp->rxsem);
    chEvtBroadcastFlagsI(&canp->rxfull_event, CAN_MAILBOX_TO_MASK(1));
    chSysUnlockFromIsr();
}

/**
 * @brief   Common error ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_err_handler(CANDriver *canp, uint32_t err_code) {
    chSysLockFromIsr();
	uint8_t rxerr = (canp->can->GSR >> 16) & 0xff;
	uint8_t txerr = (canp->can->GSR >> 24) & 0xff;
	if((rxerr > 127) || (txerr > 127)) {
		canp->can->MOD |= CAN_MOD_RM ;
		canp->can->GSR = 0;
		canp->can->MOD = 0;
	}
    /* The content of the ESR register is copied unchanged in the upper
       half word of the listener flags mask.*/
    chEvtBroadcastFlagsI(&canp->error_event, err_code);
    chSysUnlockFromIsr();
}


#if CAN_USE_SLEEP_MODE
/**
 * @brief   Wake up ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_wakeup_handler(CANDriver *canp) {

  /* Wakeup event.*/
  canp->state = CAN_READY;
  can_lld_wakeup(canp);
  chSysLockFromIsr();
  chEvtBroadcastI(&canp->wakeup_event);
  chSysUnlockFromIsr();

}
#endif /* CAN_USE_SLEEP_MODE */

/**
 * @brief   Common RX TX ERR ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_handler(void) {

#if LPC17XX_CAN_USE_CAN1 || defined(__DOXYGEN__)
	uint32_t icr1 = CAND1.can->ICR;
	if((icr1 & CAN_ICR_RI) != 0) {
		can_lld_rx_handler(&CAND1); 
	}
	if((icr1 & CAN_ICR_DOI) != 0) {
		can_lld_cdo_handler(&CAND1, icr1);
	} 
	if(icr1 & (CAN_ICR_TI1 | CAN_ICR_TI2 | CAN_ICR_TI3)) {
		can_lld_tx_handler(&CAND1); 
	}
	if(icr1 & (CAN_ICR_EI | CAN_ICR_EPI | CAN_ICR_BEI)) {
		can_lld_err_handler(&CAND1, icr1);
	}

#endif


#if LPC17XX_CAN_USE_CAN2 || defined(__DOXYGEN__)
	uint32_t icr2 = CAND2.can->ICR;
	if((icr2 & CAN_ICR_RI) != 0) {
		can_lld_rx_handler(&CAND2); 
	}
	if((icr2 & CAN_ICR_DOI) != 0) {
		can_lld_cdo_handler(&CAND2, icr2);
	} 
	if(icr2 & (CAN_ICR_TI1 | CAN_ICR_TI2 | CAN_ICR_TI3)) {
		can_lld_tx_handler(&CAND2); 
	}
	if(icr2 & (CAN_ICR_EI | CAN_ICR_EPI | CAN_ICR_BEI)) {
		can_lld_err_handler(&CAND2, icr2);
	}

#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   CAN1 TX interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(VectorA4) {

  CH_IRQ_PROLOGUE();

  can_lld_handler();

  CH_IRQ_EPILOGUE();
}

#if CAN_USE_SLEEP_MODE
/**
 * @brief   CAN Common, CAN 1 Tx, CAN 1 Rx, CAN 2 Tx, CAN 2 Rx.
 *
 * @isr
 */
CH_IRQ_HANDLER(VectorC8) {

 CH_IRQ_PROLOGUE();
#if LPC17XX_CAN_USE_CAN1 || defined(__DOXYGEN__)
 uint32_t icr1 = CAND1.can->ICR;
 if((icr1 & CAN_ICR_WUI) != 0) { 
	 can_lld_wakeup_handler(&CAND1);
 }
#endif

#if LPC17XX_CAN_USE_CAN2 || defined(__DOXYGEN__) 
 uint32_t icr2 = CAND2.can->ICR; 
 if((icr2 & CAN_ICR_WUI) != 0) { 
	 can_lld_wakeup_handler(&CAND2); 
 }
#endif

 CH_IRQ_EPILOGUE();
}
#endif /* CAN_USE_SLEEP_MODE */


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if LPC17XX_CAN_USE_CAN1
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.can = LPC_CAN1;
  LPC_SC->PCONP |=  PCCAN1;
#endif
#if LPC17XX_CAN_USE_CAN2
  /* Driver initialization.*/
  canObjectInit(&CAND2);
  CAND2.can = LPC_CAN2;
  LPC_SC->PCONP |=  PCCAN2;
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_start(CANDriver *canp) {

  /* Clock activation.*/
#if LPC17XX_CAN_USE_CAN1
  if (&CAND1 == canp) {
#ifdef LPC177x_8x
    PINSEL_ConfigPin(0,0,1); //CAN_RD1
    PINSEL_ConfigPin(0,1,1); //CAN_TD1
#else
	/* Pin P0.0 used as RD1 (CAN1) */
	LPC_PINCON->PINSEL0 |=  (1 <<  0); 

	/* Pin P0.1 used as TD1 (CAN1) */
    LPC_PINCON->PINSEL0 |=  (1 <<  2);
#endif
  }
#endif
#if LPC17XX_CAN_USE_CAN2
  if (&CAND2 == canp) {
#ifdef LPC177x_8x
    PINSEL_ConfigPin(0,4,2); //CAN_RD2
    PINSEL_ConfigPin(0,5,2); //CAN_TD2
#else
	/* Pin P0.4 used as RD2 (CAN2) */
	LPC_PINCON->PINSEL0 |=  (1 <<  9);       

	/* Pin P0.5 used as TD2 (CAN2) */
    LPC_PINCON->PINSEL0 |=  (1 << 11);  
#endif
  }
#endif

  /* Entering initialization mode. */
  canp->state = CAN_STARTING;

  /* By default filter is not used */
  LPC_CANAF->AFMR = 0x02; 

  canp->can->MOD = 1; // Enter Reset Mode
  canp->can->IER = 0; // Disable All CAN Interrupts
  canp->can->GSR = 0;

  /* Interrupt sources initialization.*/
  canp->can->IER = CAN_IER_ALL;

  /* BTR initialization.*/
  canp->can->BTR = canp->config->btr;

  /* Return to normal operating */
  canp->can->MOD = 0;
  nvicEnableVector(CAN_IRQn, CORTEX_PRIORITY_MASK(LPC17XX_CAN_IRQ_PRIORITY));

}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  /* If in ready state then disables the CAN peripheral.*/
  if (canp->state == CAN_READY) {
#if LPC17XX_CAN_USE_CAN1
    if (&CAND1 == canp) {
	  /* All sources disabled.    */
      canp->can->IER = 0x00000000;           
    }
#endif
#if LPC17XX_CAN_USE_CAN2
    if (&CAND2 == canp) {
	  /* All sources disabled.    */
      canp->can->IER = 0x00000000;
    }
#endif
    nvicDisableVector(CAN_IRQn);
  }
}

/**
 * @brief   get the index of mailbox.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @return              The index of mailbox.       
 *
 * @notapi
 */
int get_mailbox_index(CANDriver *canp) 
{
	if(canp->can->SR & CAN_SR_TBS1) {
		return 1;
	}else if(canp->can->SR & CAN_SR_TBS2) {
		return 2;
	}else if(canp->can->SR & CAN_SR_TBS3) {
		return 3;
	}
	return -1;
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool_t can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox) {

  switch (mailbox) {
  case CAN_ANY_MAILBOX:
    return (canp->can->GSR & CAN_GSR_TBS) != 0;
  case 1:
    return (canp->can->SR & CAN_SR_TBS1) != 0;
  case 2:
    return (canp->can->SR & CAN_SR_TBS2) != 0;
  case 3:
    return (canp->can->SR & CAN_SR_TBS3) != 0;
  default:
    return FALSE;
  }
}


/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp,
                      canmbx_t mailbox,
                      const CANTxFrame *ctfp) {
  int index;
  uint32_t tfi, cmr;
  CAN_TxMailBox_TypeDef *tmbp;

  /* Pointer to a free transmission mailbox.*/
  if(mailbox == CAN_ANY_MAILBOX) {
	  index = get_mailbox_index(canp);
	  if(index < 0) 
		  return;
  }else{
	  index = mailbox;
  }

  switch (index) {
	  case 1:
		cmr = 0x21;
		tmbp = &canp->can->sTxMailBox[0];
		break;
	  case 2:
		cmr = 0x41;
		tmbp = &canp->can->sTxMailBox[1];
		break;
	  case 3:
		cmr = 0x81;
		tmbp = &canp->can->sTxMailBox[2];
		break;
	  default:
		return;
  }

  /* Preparing the message.*/
  if (ctfp->IDE) {
  	tfi = ((uint32_t)ctfp->DLC << 16) | ((uint32_t)ctfp->RTR << 1) | 0x80000000;
	tmbp->TID = ctfp->EID;
  } else {
  	tfi = ((uint32_t)ctfp->DLC << 16) | ((uint32_t)ctfp->RTR << 1);
	tmbp->TID = ctfp->SID;
  }
  tmbp->TFI = tfi;
  tmbp->TDA = ctfp->data32[0];
  tmbp->TDB = ctfp->data32[1]; 

  /*Write transmission request*/
  canp->can->CMR = cmr;
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool_t can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {
	int num;
	num = mailbox;
	num = canp->rx_map.num;
	if(num > 0) {
		return TRUE;
	}else{
		return FALSE;
	}
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp,
                     canmbx_t mailbox,
                     CANRxFrame *crfp) {

	int index = mailbox;
	index = canp->rx_map.start;
	memcpy(crfp, &canp->rx_map.buf[index], sizeof(CANRxFrame));  
	canp->rx_map.start = (canp->rx_map.start + 1) & LPC17XX_CAN_RX_BUF_MASK;
	canp->rx_map.num--;

}

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  canp->can->MOD |= CAN_MOD_SM;
}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  uint32_t reg_val = 0;

#if LPC17xx_CAN_USE_CAN1
  if (&CAND1 == canp) {
    reg_val = (1UL << 1);
  }
#endif
#if LPC17xx_CAN_USE_CAN2
  if (&CAND2 == canp) {
    reg_val = (1UL << 2);
  }
#endif

  LPC_SC->CANSLEEPCLR = reg_val;
  canp->can->MOD &= ~CAN_MOD_SM;
  LPC_SC->CANWAKEFLAGS = reg_val;
}
#endif /* CAN_USE_SLEEP_MODE */

/**
 * @brief   Programs the filters.
 * @note    This is an LPC17XX-specific API.
 *
 * @param[in] can2sb    number of the first filter assigned to CAN2
 * @param[in] num       number of entries in the filters array, if zero then
 *                      a default filter is programmed
 * @param[in] cfp       pointer to the filters array, can be @p NULL if
 *                      (num == 0)
 *
 * @api
 */
void canLPC17XXSetFilters(uint32_t can2sb, uint32_t num, const CANFilter *cfp) {

  can_lld_set_filters(can2sb, num, cfp);
}

#endif /* HAL_USE_CAN */

/** @} */
