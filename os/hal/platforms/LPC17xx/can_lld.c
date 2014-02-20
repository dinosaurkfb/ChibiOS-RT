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
#if 0
  /* Temporarily enabling CAN1 clock.*/
  rccEnableCAN1(FALSE);

  /* Filters initialization.*/
  CAN1->FMR = (CAN1->FMR & 0xFFFF0000) | (can2sb << 8) | CAN_FMR_FINIT;
  if (num > 0) {
    uint32_t i, fmask;

    /* All filters cleared.*/
    CAN1->FA1R = 0;
    CAN1->FM1R = 0;
    CAN1->FS1R = 0;
    CAN1->FFA1R = 0;
    for (i = 0; i < LPC17XX_CAN_MAX_FILTERS; i++) {
      CAN1->sFilterRegister[i].FR1 = 0;
      CAN1->sFilterRegister[i].FR2 = 0;
    }

    /* Scanning the filters array.*/
    for (i = 0; i < num; i++) {
      fmask = 1 << cfp->filter;
      if (cfp->mode)
        CAN1->FM1R |= fmask;
      if (cfp->scale)
        CAN1->FS1R |= fmask;
      if (cfp->assignment)
        CAN1->FFA1R |= fmask;
      CAN1->sFilterRegister[cfp->filter].FR1 = cfp->register1;
      CAN1->sFilterRegister[cfp->filter].FR2 = cfp->register2;
      CAN1->FA1R |= fmask;
      cfp++;
    }
  }
  else {
    /* Setting up a single default filter that enables everything for both
       CANs.*/
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->sFilterRegister[can2sb].FR1 = 0;
    CAN1->sFilterRegister[can2sb].FR2 = 0;
    CAN1->FM1R = 0;
    CAN1->FFA1R = 0;
    CAN1->FS1R = 1 | (1 << can2sb);
    CAN1->FA1R = 1 | (1 << can2sb);
  }
  CAN1->FMR &= ~CAN_FMR_FINIT;

  /* Clock disabled, it will be enabled again in can_lld_start().*/
  rccDisableCAN1(FALSE);
#endif
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
static void can_lld_cdo_handler(CANDriver *canp) {
	canp->can->CMR = 0x08;
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&canp->error_event, CAN_OVERFLOW_ERROR);
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
 * @brief   Common SCE ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
#if 0
static void can_lld_sce_handler(CANDriver *canp) {
  uint32_t msr;

  msr = canp->can->MSR;
  canp->can->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;
  /* Wakeup event.*/
#if CAN_USE_SLEEP_MODE
  if (msr & CAN_MSR_WKUI) {
    canp->state = CAN_READY;
    canp->can->MCR &= ~CAN_MCR_SLEEP;
    chSysLockFromIsr();
    chEvtBroadcastI(&canp->wakeup_event);
    chSysUnlockFromIsr();
  }
#endif /* CAN_USE_SLEEP_MODE */
  /* Error event.*/
  if (msr & CAN_MSR_ERRI) {
    flagsmask_t flags;
    uint32_t esr = canp->can->ESR;

    canp->can->ESR &= ~CAN_ESR_LEC;
    flags = (flagsmask_t)(esr & 7);
    if ((esr & CAN_ESR_LEC) > 0)
      flags |= CAN_FRAMING_ERROR;

    chSysLockFromIsr();
    /* The content of the ESR register is copied unchanged in the upper
       half word of the listener flags mask.*/
    chEvtBroadcastFlagsI(&canp->error_event, flags | (flagsmask_t)(esr << 16));
    chSysUnlockFromIsr();
  }
}
#endif

#if 0
typedef struct {
	char *name;
	uint32_t v;
} RegNode;
RegNode RegMap[32];
static int sgi = 0;

void putRegMap(char *name, uint32_t v) {
	if(sgi >= 32) {
		sgi = 0;
	}
	RegMap[sgi].name = name;
	RegMap[sgi].v = v;
	sgi++;
}

void printRegMap() {
	int i;
	for(i=0; i<sgi; i++) {
		LOG_PRINT("%02d %s=0x%x.\n", i, 
				RegMap[i].name,
				RegMap[i].v
				);
	}
	LOG_PRINT("\n");
	sgi=0;
}
#endif

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
		can_lld_cdo_handler(&CAND1);
	} 
	if(((icr1 & CAN_ICR_TI1) != 0) || 
			((icr1 & CAN_ICR_TI2) != 0) ||
			((icr1 & CAN_ICR_TI3) != 0)) {
		can_lld_tx_handler(&CAND1); 
	}
#endif

#if LPC17XX_CAN_USE_CAN2 || defined(__DOXYGEN__)
	uint32_t icr2 = CAND2.can->ICR;
	if((icr2 & CAN_ICR_RI) != 0) {
		can_lld_rx_handler(&CAND2); 
	}
	if((icr2 & CAN_ICR_DOI) != 0) {
		can_lld_cdo_handler(&CAND2);
	} 
	if(((icr2 & CAN_ICR_TI1) != 0) || 
			((icr1 & CAN_ICR_TI2) != 0) ||
			((icr1 & CAN_ICR_TI3) != 0)) {
		can_lld_tx_handler(&CAND2); 
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
#endif
#if LPC17XX_CAN_USE_CAN2
  /* Driver initialization.*/
  canObjectInit(&CAND2);
  CAND2.can = LPC_CAN2;
#endif

#if 0
  /* Filters initialization.*/
#if LPC17XX_HAS_CAN2
  can_lld_set_filters(LPC17XX_CAN_MAX_FILTERS / 2, 0, NULL);
#else
  can_lld_set_filters(LPC17XX_CAN_MAX_FILTERS, 0, NULL);
#endif
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
	LPC_SC->PCONP |=  PCCAN1;
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
	LPC_SC->PCONP |=  PCCAN2;
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
	  LPC_SC->PCONP &= (~PCCAN1);
    }
#endif
#if LPC17XX_CAN_USE_CAN2
    if (&CAND2 == canp) {
	  /* All sources disabled.    */
      canp->can->IER = 0x00000000;
	  LPC_SC->PCONP &= (~PCCAN2);
    }
#endif
    nvicDisableVector(CAN_IRQn);
  }
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

  canp->can->MOD &= ~CAN_MOD_SM;
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

#if 0
  chDbgCheck((can2sb > 1) && (can2sb < LPC17XX_CAN_MAX_FILTERS) &&
             (num < LPC17XX_CAN_MAX_FILTERS),
             "canLPC17XXSetFilters");

#if LPC17XX_CAN_USE_CAN1
  chDbgAssert(CAND1.state == CAN_STOP,
              "canLPC17XXSetFilters(), #1", "invalid state");
#endif
#if LPC17XX_CAN_USE_CAN2
  chDbgAssert(CAND2.state == CAN_STOP,
              "canLPC17XXSetFilters(), #2", "invalid state");
#endif
#endif

  can_lld_set_filters(can2sb, num, cfp);
}

#endif /* HAL_USE_CAN */

/** @} */
