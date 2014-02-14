#include "ch.h"
#include "hal.h"
#include "string.h"

#if ENABLE_IAP
#include "iap.h"
#include "update.h"
#include "ver.h"

#define MAX_CFG_LEN 		16
static uint8_t s_buf[MAX_CFG_LEN];
void memdump(unsigned char *buf, size_t len);

const uint32_t update_cfg[] __attribute__ ((section(".update_cfg"))) = 
{
  UPDATE_TYPE,
  CHIP_TYPE,
  DEVICE_ID,
  CODE_VERSION,
  0,
  COM0_BAUD_RATE,
  ISP_BAUD_RATE,
  OSC_FREQ_KHZ,
  IAP_SEC_FLAG_ADDR,
  IAP_SEC_USER_ADDR,
  0, /* reserved */
  0, /* reserved */
};

void memdump(unsigned char *buf, size_t len)
{
  LOG_PRINT("###################################\n");
  uint32_t i = 0;
  for (i=0; i<len; ++i) {
    LOG_PRINT("%02x ", buf[i]);
    if (i % 16 == 15)
      LOG_PRINT("\n");
  }
  LOG_PRINT("\n");
}

// buf: ����>=16�ֽ� 
static int GetMaintainPacket(uint8_t *buf)
{
  size_t len = MAX_CFG_LEN + 1;
  uint32_t i;
  uint8_t sum;
  BaseSequentialStream  *bssp = (BaseSequentialStream  *)&SD1;

  /* Wait for a valid head and len */
  chSequentialStreamRead(bssp, buf, 2);
  /* memdump(buf, 2); */
  if (0xF5 == buf[1]) {
    s_buf[0] = buf[1];
    len = chSequentialStreamGet(bssp);
  } else if (0xF5 == buf[0]) {
    s_buf[0] = buf[0];
    len = buf[1];
  }
  if (len > MAX_CFG_LEN) {
    return 0;
  }

  s_buf[1] = len;
  /* Read the remaining bytes */
  chSequentialStreamRead(bssp, s_buf + 2, len - 2);
  /* memdump(s_buf + 2, len - 2); */

  /* Check packet's validity */
  sum = 0;
  for(i = 0; i < len-1; i++)
    sum += s_buf[i];
  /* Bad sum */
  if(sum != s_buf[len-1]) {
    return 0;
  }
  
  /* memdump(buf, len); */
  memcpy(buf, s_buf, len);

  return len;
}

void uart0_scan(void) {
  static uint32_t buf0[5];
  /* To make sure "chip, devid, ver" are on 4 byte boundary. */
  uint8_t *buf = (uint8_t *)buf0 + 1;
  BaseSequentialStream  *bssp = (BaseSequentialStream  *)&SD1;

  while(1) {
    int len = GetMaintainPacket(buf);
    if(len<=0)
      break;
    /* LOG_PRINT("start update ...\n"); */
		
    int cmd = buf[2];
    switch(cmd) {
      case TYPE_UART_UPDATA:
#if 0
	uint32_t cmd_chip, cmd_dev, cmd_ver;
	uint32_t my_chip, my_dev, my_ver;
	cmd_chip = *(uint32_t *)(buf+3);
	cmd_dev  = *(uint32_t *)(buf+7);
	cmd_ver  = *(uint32_t *)(buf+11);
	my_chip  = *(uint32_t *)64;
	my_dev   = *(uint32_t *)68;
	my_ver   = *(uint32_t *)72;
	/* LOG_PRINT("cmd_chip=%08x my_chip=%08x\n", cmd_chip, my_chip); */
	/* LOG_PRINT("cmd_dev =%08x my_dev =%08x\n", cmd_dev,  my_dev); */
	/* LOG_PRINT("cmd_ver =%08x my_ver =%08x\n", cmd_ver,  my_ver); */

	if(cmd_chip != my_chip) {
	  /* chip doesn't match */
	  chSequentialStreamWrite(bssp, "{ack2}{ack2}", 12);
	}
	else if(cmd_dev && cmd_dev != my_dev) {
	  /* device number doesn't match */
	  chSequentialStreamWrite(bssp, "{ack1}{ack1}", 12);
	}
	else if(cmd_ver && cmd_ver <= my_ver) {
	  /* no need to update */
	  chSequentialStreamWrite(bssp, "{ack0}{ack0}", 12);
	}
	else 
#endif
	  {
	    chSequentialStreamWrite(bssp, (const uint8_t *)"{ack3}{ack3}", 12);
	    chThdSleepMilliseconds(50);
	    enter_isp_with_wdt();
	  }
	break;
      }
  }
}

#endif /* #if ENABLE_IAP */
