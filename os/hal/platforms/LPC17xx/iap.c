#include "ch.h"
#include "hal.h"

#define  IAP_SELSECTOR        50
#define  IAP_RAMTOFLASH       51
#define  IAP_ERASESECTOR      52
#define  IAP_BLANKCHK         53
#define  IAP_READPARTID       54
#define  IAP_BOOTCODEID       55
#define  IAP_COMPARE          56

/* Define firmware Functions */
#define IAP_LOCATION 0x1fff1ff1
typedef void (*IAP)(uint32_t [], uint32_t[]);
IAP iap_entry = (IAP) IAP_LOCATION;

/* Input parameters for IAP */
uint32_t  paramin[8];
/* Output parameters for IAP */
uint32_t  paramout[8];

uint32_t get_iap_return(int v)
{
  return paramout[v];
}

/**
 * @brief   Select sectors
 * @note    
 *          
 * @param[in] sec1     Start Sector Number
 * @param[in] sec2     End Sector Number
 * @retval CMD_SUCCESS
 * @retval BUSY
 * @retval INVALID_SECTOR
 * @retval PARAM_ERROR
 *
 * @special
 */
void  SelSector(uint8_t sec1, uint8_t sec2)
{  
  /* set IAP command */
  paramin[0] = IAP_SELSECTOR;
  /* set parameters */
  paramin[1] = sec1;
  paramin[2] = sec2;
  iap_entry(paramin, paramout);
}

/**
 * @brief   Copy RAM to Flash
 * @note    
 *          
 * @param[in] dst      Destination flash address to be written. 
 *                     Address should be a 256 byte boundary.
 * @param[in] src      Source RAM address from where data are to be read.
 * @retval CMD_SUCCESS
 * @retval SRC_ADDR_ERROR (Address not on word boundary)
 * @retval DST_ADDR_ERROR (Address not on correct boundary)
 * @retval SRC_ADDR_NOT_MAPPED
 * @retval DST_ADDR_NOT_MAPPED
 * @retval COUNT_ERROR (Byte count is not 256 | 512 | 1024 | 4096)
 * @retval SECTOR_NOT_PREPARED_FOR WRITE_OPERATION
 * @retval BUSY
 * @retval CMD_LOCKED
 * @retval PARAM_ERROR
 * @retval CODE_READ_PROTECTION_ENABLED
 *
 * @special
 */
void  RamToFlash(uint32_t dst, uint32_t src, uint32_t no)
{
  /* set IAP command */
  paramin[0] = IAP_RAMTOFLASH;
  /* set parameters */
  paramin[1] = dst;
  paramin[2] = src;
  paramin[3] = no;
  paramin[4] = LPC17xx_CCLK/1000;
  iap_entry(paramin, paramout);
}


/**
 * @brief   Erase sector(s)
 * @note    
 *          
 * @param[in] sec1     Start Sector Number
 * @param[in] sec2     End Sector Number
 * @retval CMD_SUCCESS
 * @retval BUSY
 * @retval INVALID_SECTOR
 * @retval SECTOR_NOT_PREPARED_FOR WRITE_OPERATION
 * @retval CMD_LOCKED
 * @retval PARAM_ERROR
 * @retval CODE_READ_PROTECTION_ENABLED
 *
 * @special
 */
void  EraseSector(uint8_t sec1, uint8_t sec2)
{
  /* set IAP command */
  paramin[0] = IAP_ERASESECTOR;
  paramin[1] = sec1;
  paramin[2] = sec2;
  paramin[3] = LPC17xx_CCLK/1000;
  iap_entry(paramin, paramout);


}

/**
 * @brief   Blank check sector(s)
 * @note    
 *          
 * @param[in] sec1     Start Sector Number
 * @param[in] sec2     End Sector Number
 * @retval CMD_SUCCESS
 * @retval SECTOR_NOT_BLANK(followed by 
 *         <Offset of the first non blank word location>
 *         <Contents of non blank word location>)
 * @retval INVALID_SECTOR
 * @retval PARAM_ERROR
 *
 * @special
 */
void  BlankCHK(uint8_t sec1, uint8_t sec2)
{
  /* set IAP command */
  paramin[0] = IAP_BLANKCHK;
  paramin[1] = sec1;
  paramin[2] = sec2;
  iap_entry(paramin, paramout);
}

/**
 * @brief   Read Part Identification number
 * @note    
 *          
 * @retval CMD_SUCCESS followed by part identification number in ASCII
 *
 * @special
 */
void  ReadParID(void)
{
  paramin[0] = IAP_READPARTID;
  iap_entry(paramin, paramout);         	
}


/**
 * @brief   Read Boot Code version number
 * @note    
 *          
 * @retval CMD_SUCCESS followed by 2 bytes of boot code version number in 
 *         ASCII format. 
 *         It is to be interpreted as <byte1(Major)>.<byte0(Minor)>.
 *
 * @special
 */
void  BootCodeID(void)
{ 
  paramin[0] = IAP_BOOTCODEID;
  iap_entry(paramin, paramout);         	
}

/**
 * @brief   Compare
 * @note    
 *          
 * @param[in] dst      Starting flash or RAM address to be compared.
 *                     This address should be a word boundary.
 * @param[in] src      Starting flash or RAM address to be compared.
 *                     This address should be a word boundary.
 * @param[in] no       Number of bytes to be compared; multiple of 4.
 *
 * @retval CMD_SUCCESS (Source and destination data are equal)
 * @retval COMPARE_ERROR (Followed by the offset of first mismatch)
 * @retval COUNT_ERROR (Byte count is not a multiple of 4)
 * @retval ADDR_ERROR
 * @retval ADDR_NOT_MAPPED
 * @retval PARAM_ERROR
 * @retval CODE_READ_PROTECTION_ENABLED
 *
 * @special
 */
void  Compare(uint32_t dst, uint32_t src, uint32_t no)
{
  paramin[0] = IAP_COMPARE;
  paramin[1] = dst;
  paramin[2] = src;
  paramin[3] = no;
  iap_entry(paramin, paramout);        	
}


/* watchdog timeout value in ms */
#define WDT_TIMEOUT 500

void enter_isp_with_wdt(void)
{
  /* Set watchdog timeout */
  LPC_WDT->TC =WDT_TIMEOUT * (LPC17xx_CCLK / 40000);
  /* Enable watchdog, reset system when timeout */
  LPC_WDT->MOD=3;
  chSysDisable();

  /* Feed watchdog */
  LPC_WDT->FEED = (0xAA);
  LPC_WDT->FEED = (0x55);

  SelSector(0,0);
  EraseSector(0,0);

  volatile int i = 0;
  volatile int j = 0;
  while(1) {
    if (i++ == 0x00ffffff) {
      if (j++ == 10) {
	j = 0;
      }
      i = 0;
    }
  }
}

