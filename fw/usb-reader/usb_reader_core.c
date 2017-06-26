
#include "usb_reader_core.h"

uint8_t  usbd_reader_init(void  *pdev, uint8_t cfgidx);
uint8_t  usbd_reader_deinit      (void  *pdev, uint8_t cfgidx);
uint8_t  usbd_reader_setup       (void  *pdev, USB_SETUP_REQ *req);
uint8_t  usbd_reader_ep0_rxready  (void *pdev);
uint8_t  usbd_reader_ep0_txsent	(void* pdev);
uint8_t  usbd_READER_DATAin      (void *pdev, uint8_t epnum);
uint8_t  usbd_READER_DATAout     (void *pdev, uint8_t epnum);
uint8_t  usbd_reader_sof         (void *pdev);
static uint8_t  *usbd_reader_get_config_desc (uint8_t speed, uint16_t *length);
static uint8_t  *usbd_reader_get_usr_str_desc( uint8_t speed ,uint8_t index,  uint16_t *length);

/* USB CDC device Configuration Descriptor */
const uint8_t usbd_reader_config_dsc[USB_READER_CONFIG_DESC_SIZ] =
	{
	/*Configuration Descriptor*/
	0x09,   /* bLength: Configuration Descriptor size */
	0x02,      /* bDescriptorType: Configuration */
	USB_READER_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
	0x00,
	0x01,   /* bNumInterfaces: 1 interface */
	0x01,   /* bConfigurationValue: Configuration value */
	0x04,   /* iConfiguration: Index of string descriptor describing the configuration */
	0xC0,   /* bmAttributes: self powered */
	0x32,   /* MaxPower 0 mA */
//9
	/*---------------------------------------------------------------------------*/

	/*Interface Descriptor */
	0x09,   /* bLength: Interface Descriptor size */
	0x04,  /* bDescriptorType: Interface */
//11
	/* Interface descriptor type */
	0x00,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x02,   /* bNumEndpoints: One endpoints used */
	0xff,   /* bInterfaceClass: Communication Interface Class */
	0xff,   /* bInterfaceSubClass: Abstract Control Model */
	0xff,   /* bInterfaceProtocol: Common AT commands */
	0x05,   /* iInterface: */
//18
	/*Serial number endpoint OUT Descriptor*/
	0x07,  /* bLength: Endpoint Descriptor size */
	0x05,  /* bDescriptorType: Endpoint */
	READER_DATA_IN_EP,  /* bEndpointAddress */
	0x02,  /* bmAttributes: Bulk */
	USB_READER_EP_DATA_SIZE,  /* wMaxPacketSize: */
	0x00,
	0x00,  /* bInterval: ignore for Bulk transfer */
//25
	/* Decrypted data endpoint OUT Descriptor */
	0x07,  /* bLength: Endpoint Descriptor size */
	0x05,  /* bDescriptorType: Endpoint */
	READER_DATA_OUT_EP,  /* bEndpointAddress */
	0x02,  /* bmAttributes: Bulk */
	USB_READER_EP_DATA_SIZE,  /* wMaxPacketSize: */
	0x00,
	0x00  /* bInterval: ignore for Bulk transfer */
//32
	};

USBD_Class_cb_TypeDef  USBD_reader_cb =
	{
	usbd_reader_init,
	usbd_reader_deinit,
	usbd_reader_setup,
	usbd_reader_ep0_txsent,                 /* EP0_TxSent, */
	usbd_reader_ep0_rxready,
	usbd_READER_DATAin,
	usbd_READER_DATAout,
	usbd_reader_sof,
	usbd_reader_get_config_desc,
	usbd_reader_get_usr_str_desc,
	};

uint8_t  usbd_reader_init (void  *pdev,  uint8_t cfgidx)
	{
	DCD_PMA_Config(pdev , READER_DATA_IN_EP,USB_SNG_BUF,READER_DATA_IN_ADDRESS);
	DCD_PMA_Config(pdev , READER_DATA_OUT_EP,USB_SNG_BUF,READER_DATA_OUT_ADDRESS);


	DCD_EP_Open(pdev,READER_DATA_IN_EP,USB_READER_EP_DATA_SIZE,USB_EP_BULK);
	DCD_EP_Open(pdev,READER_DATA_OUT_EP,USB_READER_EP_DATA_SIZE,USB_EP_BULK);

	DCD_EP_PrepareRx(pdev, READER_DATA_OUT_EP, (uint8_t*)(reader_temp_buffer),USB_READER_EP_DATA_SIZE);
	return USBD_OK;
	}

uint8_t  usbd_reader_deinit (void  *pdev, uint8_t cfgidx)
	{
	DCD_EP_Close(pdev,READER_DATA_IN_EP);
	DCD_EP_Close(pdev,READER_DATA_OUT_EP);
	return USBD_OK;
	}

uint8_t  usbd_reader_setup (void  *pdev,USB_SETUP_REQ *req)
	{

	uint16_t len= USB_READER_DESC_SIZ;
	uint8_t  *pbuf= (uint8_t*)usbd_reader_config_dsc + 9;

	switch (req->bmRequest & USB_REQ_TYPE_MASK)
		{
		case USB_REQ_TYPE_VENDOR:
			{
			if (req->bmRequest & 0x80)
				{
				USBD_CtlSendData (pdev,(uint8_t*)&usb_reader_cmd,sizeof(usb_reader_cmd));
				}
			else
				{
				if(req->wLength == 2)
					{
					USBD_CtlPrepareRx (pdev,(uint8_t*)&usb_reader_cmd,sizeof(usb_reader_cmd));
					}
				}
			};
		break;

		case USB_REQ_TYPE_CLASS :
		default:
			USBD_CtlError (pdev, req);
			return USBD_FAIL;

		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest)
				{
				case USB_REQ_GET_DESCRIPTOR:
					USBD_CtlSendData (pdev,pbuf,len);
					break;

				case USB_REQ_GET_INTERFACE:
				case USB_REQ_SET_INTERFACE:
					USBD_CtlError (pdev, req);
					return USBD_FAIL;
				}
		}
	return USBD_OK;
	}

uint8_t  usbd_reader_ep0_rxready (void  *pdev)
	{
	return USBD_OK;
	}

uint8_t  usbd_READER_DATAin (void *pdev, uint8_t epnum)
	{
	DCD_EP_Tx (pdev,READER_DATA_IN_EP,reader_temp_buffer,(usb_reader_cmd>>8));
	return USBD_OK;
	}

/**
  * @brief  usbd_READER_DATAout
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  usbd_READER_DATAout (void *pdev, uint8_t epnum)
	{
	DCD_EP_PrepareRx(pdev, READER_DATA_OUT_EP, (uint8_t*)(reader_temp_buffer),(usb_reader_cmd>>8));
	return USBD_OK;
	}

/**
  * @brief  usbd_CDC_SOF
  *         Start Of Frame event management
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  usbd_reader_sof  (void *pdev)
	{
	DCD_EP_Tx (pdev,READER_DATA_IN_EP,reader_temp_buffer,(usb_reader_cmd>>8) );
	return USBD_OK;
	}

/**
  * @brief  usbd_reader_get_config_desc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *usbd_reader_get_config_desc (uint8_t speed, uint16_t *length)
	{
	*length = sizeof (usbd_reader_config_dsc);
	return (uint8_t*)usbd_reader_config_dsc;
	}

static uint8_t  *usbd_reader_get_usr_str_desc( uint8_t speed ,uint8_t index,  uint16_t *length)
	{
	if (index == 6)
		{
		uint8_t text[17] = "intx82@gmail.com\0";
		USBD_GetString ((uint8_t *)text, USBD_StrDesc, length);
		return USBD_StrDesc;
		}
	else
		{
		*length = 0;
		return NULL;
		}
	}

uint8_t  usbd_reader_ep0_txsent	(void* pdev)
	{
	return USBD_OK;
	}

