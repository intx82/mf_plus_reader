/**
  ******************************************************************************
  * @file    usbd_hid_core.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    31-January-2014
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Joystick
  *             - Collection : Application
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_core.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint8_t  USBD_HID_Init (void  *pdev,
							   uint8_t cfgidx);

static uint8_t  USBD_HID_DeInit (void  *pdev,
								 uint8_t cfgidx);

static uint8_t  USBD_HID_Setup (void  *pdev,
								USB_SETUP_REQ *req);

static uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length);
static uint8_t  *USBD_HID_GetUsrString (uint8_t a, uint8_t b, uint16_t *length);

USBD_Class_cb_TypeDef  USBD_HID_cb =
	{
	USBD_HID_Init,
	USBD_HID_DeInit,
	USBD_HID_Setup,
	NULL, /*EP0_TxSent*/
	NULL, /*EP0_RxReady*/
	NULL, /*DataIn*/
	NULL, /*DataOut*/
	NULL, /*SOF */
	USBD_HID_GetCfgDesc,
	USBD_HID_GetUsrString,
	};

static uint32_t  USBD_HID_AltSet = 0;

static uint32_t  USBD_HID_Protocol = 0;

static uint32_t  USBD_HID_IdleState  = 0;

uint8_t Report_buf[5];
uint8_t USBD_HID_Report_ID=0;


/* USB HID device Configuration Descriptor */
const uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] =
	{
	0x09, /* bLength: Configuration Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
	USB_HID_CONFIG_DESC_SIZ,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
	0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

	/************** Descriptor of Joystick Mouse interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x01,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: HID*/
	0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/
	/******************** Descriptor of Joystick Mouse HID ********************/
	/* 18 */
	0x09,         /*bLength: HID Descriptor size*/
	HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x11,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	HID_KEYBOARD_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
	0x00,
	/******************** Descriptor of Mouse endpoint ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

	HID_IN_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	HID_IN_PACKET, /*wMaxPacketSize: 4 Byte max */
	0x00,
	0x0A,          /*bInterval: Polling Interval (10 ms)*/
	/* 34 */
	} ;

const uint8_t HID_KEYBAORD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] =
	{
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,                    // USAGE (Keyboard)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
	0x95, 0x05,                    //   REPORT_COUNT (5)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x05, 0x08,                    //   USAGE_PAGE (LEDs)
	0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
	0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
	0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x03,                    //   REPORT_SIZE (3)
	0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
	0x95, 0x06,                    //   REPORT_COUNT (6)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION
	};



/* Private function ----------------------------------------------------------*/
/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_Init (void  *pdev,
							   uint8_t cfgidx)
	{
	DCD_PMA_Config(pdev , HID_IN_EP,USB_SNG_BUF,HID_IN_TX_ADDRESS);

	/* Open EP IN */
	DCD_EP_Open(pdev,
				HID_IN_EP,
				HID_IN_PACKET,
				USB_EP_INT);

	/* Open EP OUT */
	DCD_EP_Open(pdev,
				HID_OUT_EP,
				HID_OUT_PACKET,
				USB_EP_INT);

	return USBD_OK;
	}

/**
  * @brief  USBD_HID_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_DeInit (void  *pdev,
								 uint8_t cfgidx)
	{
	/* Close HID EPs */
	DCD_EP_Close (pdev , HID_IN_EP);
	DCD_EP_Close (pdev , HID_OUT_EP);


	return USBD_OK;
	}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup (void  *pdev,
								USB_SETUP_REQ *req)
	{
	uint16_t len = 0;
	uint8_t  *pbuf = NULL;
	uint8_t USBD_HID_Report_LENGTH = 0;
		
	switch (req->bmRequest & USB_REQ_TYPE_MASK)
		{
		case USB_REQ_TYPE_CLASS :
			switch (req->bRequest)
				{
				case HID_REQ_SET_PROTOCOL:
					USBD_HID_Protocol = (uint8_t)(req->wValue);
					break;

				case HID_REQ_GET_PROTOCOL:
					USBD_CtlSendData (pdev,
									  (uint8_t *)&USBD_HID_Protocol,
									  1);
					break;

				case HID_REQ_SET_IDLE:
					USBD_HID_IdleState = (uint8_t)(req->wValue >> 8);
					break;

				case HID_REQ_GET_IDLE:
					USBD_CtlSendData (pdev,
									  (uint8_t *)&USBD_HID_IdleState,
									  1);
					break;

				case HID_REQ_SET_REPORT:
						{
						USBD_HID_Report_ID = (uint8_t)(req->wValue);
						USBD_HID_Report_LENGTH = (uint8_t)(req->wLength);
						USBD_CtlPrepareRx (pdev, Report_buf, USBD_HID_Report_LENGTH);
						}
					break;

				default:
					USBD_CtlError (pdev, req);
					return USBD_FAIL;
				}
			break;

		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest)
				{
				case USB_REQ_GET_DESCRIPTOR:
					if( req->wValue >> 8 == HID_REPORT_DESC)
						{
						len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE , req->wLength);
						pbuf = (uint8_t *)HID_KEYBAORD_ReportDesc;
						}
					else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
						{
						pbuf = (uint8_t *)USBD_HID_CfgDesc + 0x12;
						len = MIN(USB_HID_DESC_SIZ , req->wLength);
						}

					USBD_CtlSendData (pdev,
									  pbuf,
									  len);

					break;

				case USB_REQ_GET_INTERFACE :
					USBD_CtlSendData (pdev,
									  (uint8_t *)&USBD_HID_AltSet,
									  1);
					break;

				case USB_REQ_SET_INTERFACE :
					USBD_HID_AltSet = (uint8_t)(req->wValue);
					break;
				}
		}
	return USBD_OK;
	}

/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport     (USB_CORE_HANDLE  *pdev,
								 uint8_t *report,
								 uint16_t len)
	{
	/* Check if USB is configured */
	if (pdev->dev.device_status == USB_CONFIGURED )
		{
		DCD_EP_Tx (pdev, HID_IN_EP, report, len);
		}
	return USBD_OK;
	}

/**
  * @brief  USBD_HID_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length)
	{
	*length = sizeof (USBD_HID_CfgDesc);
	return (uint8_t *)USBD_HID_CfgDesc;
	}


static uint8_t  *USBD_HID_GetUsrString (uint8_t a, uint8_t b, uint16_t *length)
	{
	*length = 0;
	return 0;
	}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
