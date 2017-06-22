#ifndef __USB_CONF__H__
#define __USB_CONF__H__
#include "stm32f0xx.h"

#define INTERNAL_PULLUP

#define EP_NUM     (2)  /* EP0 + EP1 IN For HID */
#define BTABLE_ADDRESS      (0x000)
#define ENDP0_RX_ADDRESS   (0x40)
#define ENDP0_TX_ADDRESS   (0x80)


#endif 
