
#include "usb_dev/usbd_desc.h"

#define READER_DATA_IN_EP 0x81
#define READER_DATA_OUT_EP 0x1
#define USB_READER_EP_DATA_SIZE 128
#define USB_READER_CONFIG_DESC_SIZ                (32)

#define READER_DATA_IN_ADDRESS 0xc0
#define READER_DATA_OUT_ADDRESS (READER_DATA_IN_ADDRESS+USB_READER_CONFIG_DESC_SIZ)

#pragma anon_unions
#pragma pack(push,1)

#define USB_READER_DESC_SIZ                       (USB_READER_CONFIG_DESC_SIZ -9)

typedef enum
	{
	IDLE = 0,
	REQA_WUPA,
	SELECT,
	COMM,
	STOP_CRYPTO1,
	MF_AUTH,
	READER_SERIAL
	} usb_reader_cmd_num_t;

typedef struct
	{
	union
		{
		usb_reader_cmd_num_t cmd;
		uint8_t resp;
		};
	uint8_t len;
	} usb_reader_cmd_t;

extern USBD_Class_cb_TypeDef  USBD_reader_cb;
extern uint8_t reader_temp_buffer[USB_READER_EP_DATA_SIZE];
extern uint16_t usb_reader_cmd;
extern usb_reader_cmd_t* usb_reader_cmd_ptr;
#pragma pack(pop)
