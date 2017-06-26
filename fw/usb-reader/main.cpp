#include <stm32f0xx.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../spi.hpp"
#include "../mfrc522.hpp"
#include "../aes.hpp"

volatile uint32_t sys_time = 0; /**< Глобальное время в мс */

AES_T aes;
SPI_T spi;
MFRC522_T reader(&spi, &aes);

extern "C"
	{
#include "usb_dev/usb_dcd_int.h"
#include "usb_dev/usbd_keyb_usr.h"
#include "usb_reader_core.h"
#include "usb_dev/usbd_desc.h"
#include "sha256.h"

	uint8_t reader_temp_buffer[128] = {};
	USB_CORE_HANDLE  USB_Device_dev ; /**< Описание USB устроства */
	uint16_t usb_reader_cmd;
	usb_reader_cmd_t* usb_reader_cmd_ptr;
	/**
	* Прерывание системного таймера (1мс)
	*/
	void SysTick_Handler(void)
		{
		sys_time++;
		reader.delay_cb();
		}

	/**
	*Прерывание USB
	*/
	void USB_IRQHandler(void)
		{
		USB_Istr();
		}

	/**
	*Пиздец
	*/
	void HardFault_Handler(void)
		{
		while(1);
		}
	}

/**
* Задержка в мс
* @param Кол-во милисекунд
*/
void delay(uint32_t del)
	{
	volatile uint32_t old_time = sys_time + del ;
	while(old_time > sys_time);
	}


/**
*  int main()
*/
int main()
	{
	SystemInit();
	if(SysTick_Config(48000)!=0)
		{
		return 0;
		}
	usb_reader_cmd_ptr = (usb_reader_cmd_t*) &usb_reader_cmd;
	const uint8_t* hw_serial = (uint8_t*)0x1FFFF7AC;
	delay(1000);
	reader.PCD_Init();
	USBD_Init(&USB_Device_dev,&USR_Keyb_desc,&USBD_reader_cb,&USR_Keyb_cb);

	while(1)
		{
		switch(usb_reader_cmd_ptr->cmd)
			{
			case REQA_WUPA:
				{
				uint8_t resp_len = sizeof(reader_temp_buffer);
				usb_reader_cmd_ptr->resp = (uint8_t)reader.PICC_REQA_or_WUPA(MFRC522_T::PICC_CMD_REQA,(uint8_t*)&reader_temp_buffer, &resp_len)|0x80;
				if ((usb_reader_cmd_ptr->resp&0x7f)  != MFRC522_T::STATUS_OK)
					{
					usb_reader_cmd_ptr->len = 0;
					}
				else
					{
					usb_reader_cmd_ptr->len = resp_len;
					}
				};
			break;

			case SELECT:
				{
				usb_reader_cmd_ptr->resp = (uint8_t) reader.PICC_Select((MFRC522_T::uid_t*)&reader_temp_buffer, 0)|0x80;
				if ((usb_reader_cmd_ptr->resp&0x7f)  != MFRC522_T::STATUS_OK)
					{
					usb_reader_cmd_ptr->len = 0;
					}
				else
					{
					usb_reader_cmd_ptr->len = sizeof(MFRC522_T::uid_t);
					}
				};
			break;

			case COMM:
				{
				NVIC_DisableIRQ(USB_IRQn);
				if ((reader.PCD_ReadRegister(MFRC522_T::TxModeReg)&0x80) != 0x80)
					{
					usb_reader_cmd_ptr->resp  = reader.PCD_CalculateCRC(reader_temp_buffer,usb_reader_cmd_ptr->len,&reader_temp_buffer[usb_reader_cmd_ptr->len])|0x80;
					if ((usb_reader_cmd_ptr->resp&0x7f)  != MFRC522_T::STATUS_OK)
						{
						break;
						}
					usb_reader_cmd_ptr->len += 2;
					}
				uint8_t resp_len = sizeof(reader_temp_buffer);
				usb_reader_cmd_ptr->resp = (uint8_t)reader.PCD_TransceiveData(reader_temp_buffer,usb_reader_cmd_ptr->len,reader_temp_buffer,&resp_len)|0x80;
				if ((usb_reader_cmd_ptr->resp&0x7f)  != MFRC522_T::STATUS_OK)
					{
					usb_reader_cmd_ptr->len = 0;
					}
				else
					{
					usb_reader_cmd_ptr->len = resp_len;
					}
				NVIC_EnableIRQ(USB_IRQn);
				};
			break;

			case STOP_CRYPTO1:
				{
				reader.PCD_StopCrypto1();
				usb_reader_cmd_ptr->resp = 0x80;
				usb_reader_cmd_ptr->len = 0;
				};
			break;

			case MF_AUTH:
				{
				if(usb_reader_cmd_ptr->len >= (1+sizeof(MFRC522_T::MIFARE_Key)+sizeof(MFRC522_T::uid_t)))
					{
					NVIC_DisableIRQ(USB_IRQn);
					struct mf_auth_cmd_t
						{
						uint8_t block;
						MFRC522_T::MIFARE_Key key;
						MFRC522_T::uid_t uid;
						};
					mf_auth_cmd_t* d = (mf_auth_cmd_t*)&reader_temp_buffer;
						
						
					MFRC522_T::MIFARE_Key k = {};
					memcpy((uint8_t*)&k,&d->key,6);
					MFRC522_T::uid_t s = {};
					memcpy((uint8_t*)&s,&d->uid,sizeof(MFRC522_T::uid_t));
					
					usb_reader_cmd_ptr->resp = (uint8_t)reader.PCD_Authenticate(MFRC522_T::PICC_CMD_MF_AUTH_KEY_A,d->block,&k,&s)|0x80;
					usb_reader_cmd_ptr->len = 0;
					NVIC_EnableIRQ(USB_IRQn);
					}
				else
					{
					usb_reader_cmd_ptr->resp = 0xfe;
					usb_reader_cmd_ptr->len = 0;
					}
				};
			break;

			case READER_SERIAL:
				{
				memcpy(reader_temp_buffer,hw_serial,12);
				usb_reader_cmd_ptr->resp = 0x80;
				usb_reader_cmd_ptr->len = 12;
				};
			break;

			default:
				{
				};
			}
		};
	}
