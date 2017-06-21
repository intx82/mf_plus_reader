#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "lcd/ili9488.hpp" 
#include "sd_card_error_img.h"
#include "spi.hpp"
#include "aes.hpp"
#include "../MFRC522.hpp"

static const uint8_t mfp_master_key[] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}; /**< 16*0xff */

struct std::__FILE
	{
	int handle;
	uint16_t x;
	uint16_t y;
	uint16_t offset_x;
	uint16_t offset_y;
	uint32_t text_color;
	uint32_t bg_color;
	};

FILE std::__stdout;
volatile uint32_t sys_time=0;
LCD_T lcd;
AES_T aes;
SPI_T spi;
MFRC522_T reader(&spi, &aes);

extern "C"
	{
	void SysTick_Handler(void)
		{
		sys_time++;
		reader.delay_cb();
		}
	}

int std::fputc(int ch, FILE *f)
	{
	switch(ch)
		{
		case '\r':
			{
			f->offset_x = 0;

			};
		break;
		case '\n':
			{
			f->offset_y +=9;
			};
		break;
		default:
			{
			lcd.putchar(f->x+f->offset_x,f->y+f->offset_y,ch,f->bg_color,f->text_color);
			f->offset_x += 6;
			lcd.show_rect(f->x+f->offset_x,f->x+f->offset_x+18,f->y+f->offset_y,f->y+f->offset_y+8,f->bg_color);
			}
		}


	return ch;
	}

void set_stdout_params(uint16_t x, uint16_t y, uint32_t bg_color, uint32_t text_color)
	{
	stdout->bg_color = bg_color;
	stdout->text_color = text_color;
	stdout->offset_x = 0;
	stdout->offset_y = 0;
	stdout->x = x;
	stdout->y = y;
	}

void HardFault_Handler(void)
	{
	lcd.show_rect(0,lcd.WIDTH,0,lcd.HEIGHT,0x800000);
	}

void delay(uint32_t d)
	{
	volatile uint32_t del = sys_time+d;
	while(sys_time!=del);
	}

void print_array(uint8_t* a,uint8_t len)
	{
	uint8_t i  = 0;
	while(len)
		{
		printf("%02x",(*a++));
		i++;
		if(i==16)
			{
			printf("\r\n");
			i = 0;
			}
		len--;
		if(len>0)
			{
			printf(":");
			}
		}
	}

void uart_init(uint16_t speed)
	{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB  , ENABLE);

	GPIO_InitTypeDef gpio_port;
	gpio_port.GPIO_Pin = 0x3FF;
	gpio_port.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio_port);
	GPIOB->ODR=0x00;
	gpio_port.GPIO_Pin   = GPIO_Pin_10;
	gpio_port.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_port);
	gpio_port.GPIO_Pin   = GPIO_Pin_9;
	gpio_port.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_port.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio_port);
	USART_InitTypeDef uart_struct;
	uart_struct.USART_BaudRate            = speed;
	uart_struct.USART_WordLength          = USART_WordLength_8b;
	uart_struct.USART_StopBits            = USART_StopBits_1;
	uart_struct.USART_Parity              = USART_Parity_No ;
	uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &uart_struct);
	USART_Cmd(USART1, ENABLE);
	}

void uart_tx_byte(uint8_t data)
	{
	while(!(USART1->SR & USART_SR_TC));
	USART1->DR=data;
	}

uint8_t uart_rx_byte()
	{
	while (!(USART1->SR & USART_SR_RXNE));
	return USART1->DR;
	}

bool last_pcd = false;


void handle_mifare()
	{
	MFRC522_T::uid_t mifare_serial = {};
	uint16_t mifare_card_type = 0;
	uint8_t mifare_card_type_len = sizeof(mifare_card_type);
	if (reader.PICC_REQA_or_WUPA(MFRC522_T::PICC_CMD_REQA,(uint8_t*)&mifare_card_type, &mifare_card_type_len) == MFRC522_T::STATUS_OK)
		{
		if (reader.PICC_Select(&mifare_serial, 0) == MFRC522_T::STATUS_OK)
			{
			lcd.clear_screen();
			set_stdout_params(8,8,0xffffff,0);
			printf("Card: %04x\r\n",mifare_card_type);
			printf("SAK: %02x\r\n",mifare_serial.sak);
			printf("Serial: [%02x] ",mifare_serial.size);
			print_array((uint8_t*)&mifare_serial.uidByte,sizeof(mifare_serial.uidByte));
			printf("\r\n");
			switch(mifare_serial.sak )
				{
				case 0x08: //m1s50
					{
					if(mifare_card_type != 0x44)
						{
						printf("Found MF-Classic\r\n");
						MFRC522_T::MIFARE_Key key = {0xff,0xff,0xff,0xff,0xff,0xff};
						MFRC522_T::status_t res = reader.PCD_Authenticate(MFRC522_T::PICC_CMD_MF_AUTH_KEY_A,0,&key,&mifare_serial);
						printf("Auth [ff,ff,ff,ff,ff,ff]: %x\r\n",res);
						if(res == MFRC522_T::STATUS_OK)
							{
							uint8_t buffer[18] = {};
							uint8_t size = sizeof(buffer);
							res = reader.MIFARE_Read(1, buffer, &size);
							if (res == MFRC522_T::STATUS_OK)
								{
								printf("data:\r\n");
								print_array(buffer,16);

								}
							}
						printf("Re-read in:\r\n");
						for(uint8_t i = 0; i<3; i++)
							{
							printf("%d.. ",3-i);
							delay(1000);
							}
						reader.PCD_StopCrypto1();
						}
					else
						{
						uint8_t resp[32] = {};
						uint8_t resp_len = sizeof(resp);
						MFRC522_T::status_t res = reader.RATS(resp,&resp_len);
						printf("RATS !%x ",res);
						if(res != MFRC522_T::STATUS_OK)
							{
							printf("\r\n");
							break;
							}
						print_array(resp,resp_len);
						printf("\r\n");
						printf("Found L1 card\r\n");
						printf("Delay 3 sec start init to L3\r\nRemove card NOW if init not needed\r\n");
						for(uint8_t i = 0; i<3; i++)
							{
							printf("%d.. ",3-i);
							delay(1000);
							}
						printf("\r\n");
						printf("Toggle to L3 Auth: ");
						resp_len = sizeof(resp);
						res = reader.mfp_first_auth(0x9003,(uint8_t*)mfp_master_key);
						printf("%x \r\n" ,res);

						}
					};
				break;
				case 0x20: //iso14443-4
				case 0x11:
					{
					/**< RATS!? */
					uint8_t resp[32] = {};
					uint8_t resp_len = sizeof(resp);
					MFRC522_T::status_t res = reader.RATS(resp,&resp_len);
					printf("RATS !%x ",res);
					if(res != MFRC522_T::STATUS_OK)
						{
						printf("\r\n");
						break;
						}
					print_array(resp,resp_len);
					printf("\r\n");

					printf("First Auth: ");
					resp_len = sizeof(resp);
					res = reader.mfp_first_auth(0x4000,(uint8_t*)mfp_master_key);
					printf("%x \r\n" ,res);
					if(res == 0xc9)
						{
						printf("Found L0 card\r\n");
						printf("Delay 3 sec start init to L1\r\nRemove card NOW if init not needed\r\n");
						for(uint8_t i = 0; i<3; i++)
							{
							printf("%d.. ",3-i);
							delay(1000);
							}
						printf("\r\n");
						printf("Write perso 4000-403f: ");
						for(uint16_t i = 0x4000; i < 0x403f; i+=2)
							{
							resp_len = sizeof(resp);
							res = reader.mfp_write_perso(resp,&resp_len,i,(uint8_t*)mfp_master_key);
							if((res != MFRC522_T::STATUS_OK)||(resp[0] != 0x90))
								{
								printf("%x -> %d:%x\r\n",i,res,resp[0]);
								return;
								}
							}
						printf("OK\r\n");

						printf("Write perso 9000-9003: ");
						for(uint16_t i = 0x9000; i <= 0x9003; i++)
							{
							if(i!=0x9002)
								{
								resp_len = sizeof(resp);
								res = reader.mfp_write_perso(resp,&resp_len,i,(uint8_t*)mfp_master_key);
								if((res != MFRC522_T::STATUS_OK)||(resp[0] != 0x90))
									{
									printf("err: %x -> %d:%x\r\n",i,res,resp[0]);
									return;
									}
								}
							}
						printf("OK\r\n");
						/**
						*
						*/
						printf("Write sector 1: ");
						uint8_t data[16] = {sys_time&0xff, (sys_time>>8)&0xff,(sys_time>>16)&0xff,(sys_time>>24)&0xff,0x17,0x06,0x16,0x27,0x41,0x0};
						res = reader.mfp_write_perso(resp,&resp_len,1,(uint8_t*)data);
						if((res != MFRC522_T::STATUS_OK)||(resp[0] != 0x90))
							{
							printf("err: -> %d:%x\r\n",res,resp[0]);
							return;
							}
						printf("OK\r\n");

						printf("Commit: ");
						resp_len = sizeof(resp);
						res = reader.mfp_commit_perso(resp, &resp_len);
						if((res != MFRC522_T::STATUS_OK)||(resp[0] != 0x90))
							{
							printf("err: -> %d:%x\r\n",res,resp[0]);
							return;
							}
						printf("OK\r\n");

						}
					else
						{
						/**
						* Read -> +1 -> write;

						printf("Write block: ");
						uint8_t data[16] = {0x12,0x34,0x56,0x78,0x9a,0xbc,0xef};
						res = reader.mfp_write_block(1,data,sizeof(data));
						printf("%x \r\n" ,res);
						*/
						if(res == MFRC522_T::STATUS_OK)
							{
							printf("Found L3 Card\r\n");
							printf("Card already initialized\r\n");
							delay(10);
							resp_len = sizeof(resp);
							res = reader.mfp_read_block(1,resp,&resp_len);
							if(res != MFRC522_T::STATUS_OK)
								{
								printf("err: -> %x\r\n",res);
								return;
								}
							printf("data:\r\n");
							print_array(resp,resp_len);
							printf("\r\n");
							}
						else
							{
							printf("Found Unknown ISO14443-4 Card\r\n");
							}
						}

					};
				break;
				default:
					{
					printf("Found Unknown ISO14443-3 Card\r\n");
					};
				}
			reader.PICC_HaltA();
			}
		}
	}

int main(void)
	{
	SystemInit();
	if(SysTick_Config(72000)!=0)
		{
		return 0;
		}
	spi.init();
	lcd.init();
	lcd.clear_screen();
	set_stdout_params(8,8,0xffffff,0);
	printf("Loading..\r\n");
	delay(1000);
	reader.PCD_Init();
	printf("Reader init ok\r\n");
	delay(1000);
	while(1)
		{
		handle_mifare();
		delay(100);
		}
	}
