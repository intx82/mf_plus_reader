#include <stm32f0xx.h>
#include "ow.hpp"
#include "../aes.hpp"
#include "weigand.hpp"
#include "../spi.hpp"
#include "../MFRC522.hpp"

#define D0_PIN 9
#define D1_PIN 10

volatile uint32_t sys_time = 0;

static const uint8_t err_data[] = {0,0,0xff,0xff,0xff,0xff};
static const uint8_t mfp_master_key[] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}; /**< 16*0xff */

enum connection_state_t {WEIGAND, OWI};
connection_state_t connection;

AES_T aes;
SPI_T spi;
OW_SLAVE_T ow;
WEIGAND_SLAVE_T weigand;
MFRC522_T reader(&spi,&aes);

void delay(uint32_t d)
	{
	volatile uint32_t del = sys_time+d;
	while(sys_time!=del);
	}


extern "C"
	{
	/**
	* OVF happend
	*/
	void TIM3_IRQHandler  (void)
		{
		switch(connection)
			{
			case OWI:
				ow.timeout_handler();
				break;
			case WEIGAND:
				weigand.timer_handler();
				break;
			default:
				return;
			}

		}

	void EXTI4_15_IRQHandler (void)
		{
		ow.pin_interrupt_handler();
		}

	void SysTick_Handler(void)
		{
		sys_time++;
		reader.delay_cb();
		}

	}
/**
* Проверяет подключение TRUE - OW; Иначе Weigand
*/
connection_state_t check_ow_weigand()
	{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	uint32_t old_moder = GPIOA->MODER;
	connection_state_t out = WEIGAND;
	GPIOA->MODER |= (1<<(D0_PIN<<1));
	GPIOA->MODER &= ~(1<<(D1_PIN<<1));
	GPIOA->ODR |= (1<<D0_PIN);
	delay(1);
	if(GPIOA->IDR & (1<<D1_PIN))
		{
		GPIOA->ODR &= ~(1<<D0_PIN);
		delay(1);
		if(!(GPIOA->IDR & (1<<D1_PIN)))
			{
			GPIOA->MODER |= (1<<(D1_PIN<<1));
			GPIOA->MODER &= ~(1<<(D0_PIN<<1));
			GPIOA->ODR |= (1<<D1_PIN);
			delay(1);
			if(GPIOA->IDR & (1<<D0_PIN))
				{
				GPIOA->ODR &= ~(1<<D1_PIN);
				delay(1);
				if(!(GPIOA->IDR & (1<<D0_PIN)))
					{
					out = OWI;
					}
				}
			}
		}
	GPIOA->ODR = 0;
	GPIOA->MODER = old_moder;
	return out;
	}


/**
* Функция передачи на OW/WEIGAND хеша шаблона
*/
void connection_serial_transmit(uint8_t error_code, uint8_t* data)
	{
	switch(connection)
		{
		case OWI:
			{
			uint8_t ow_data[8] = {error_code,data[0],data[1],data[2],data[3],data[4],data[5]};
			ow_data[7] = ow.crc(ow_data,7);
			ow.set_serial(ow_data,false);
			}
		break;
		case WEIGAND:
			{
			uint8_t weigand_buf[5] = {error_code, data[0],data[1],data[2],data[3]};
			for(uint8_t i = 0; i<3; i++)
				{
				weigand.send(weigand_buf);
				delay(300);
				}
			};
		break;
		default:
			return;
		}

	}

int main()
	{
	SystemInit();
	if(SysTick_Config(48000)!=0)
		{
		return 0;
		}
	delay(100);
	reader.PCD_Init();
	delay(100);
	connection = check_ow_weigand();
	switch(connection)
		{
		case OWI:
			ow.init();
			break;
		case WEIGAND:
			weigand.init();
			break;
		default:
			while(1);
		}

	while(1)
		{
		MFRC522_T::uid_t mifare_serial = {};
		uint16_t mifare_card_type = 0;
		uint8_t mifare_card_type_len = sizeof(mifare_card_type);
		if (reader.PICC_REQA_or_WUPA(MFRC522_T::PICC_CMD_REQA,(uint8_t*)&mifare_card_type, &mifare_card_type_len) == MFRC522_T::STATUS_OK)
			{
			if (reader.PICC_Select(&mifare_serial, 0) == MFRC522_T::STATUS_OK)
				{
				switch(mifare_serial.sak )
					{
					case 0x08: //m1s50
						{
						connection_serial_transmit(0,(uint8_t*)&mifare_serial.uidByte);
						/*
						MFRC522_T::MIFARE_Key key = {0xff,0xff,0xff,0xff,0xff,0xff};
						MFRC522_T::status_t res = reader.PCD_Authenticate(MFRC522_T::PICC_CMD_MF_AUTH_KEY_A,0,&key,&mifare_serial);
						printf("Auth: %d\r\n",res);
						if(res == MFRC522_T::STATUS_OK)
							{
							uint8_t buffer[18] = {};
							uint8_t size = sizeof(buffer);
							res = reader.MIFARE_Read(1, buffer, &size);
							if (res == MFRC522_T::STATUS_OK)
								{
								printf("b:{\"");
								print_array(buffer,sizeof(buffer));
								printf("\"}\r\n");
								}
							}
						reader.PCD_StopCrypto1();
							*/
						};
					break;
					case 0x20: //iso14443-4
					case 0x11:
						{
						/**< RATS!? */
						uint8_t resp[32] = {};
						uint8_t resp_len = sizeof(resp);
						MFRC522_T::status_t res = reader.RATS(resp,&resp_len);

						if(res != MFRC522_T::STATUS_OK)
							{
							connection_serial_transmit(1,(uint8_t*)err_data);
							break;
							}

						resp_len = sizeof(resp);
						res = reader.mfp_first_auth(0x4000,(uint8_t*)mfp_master_key);
						if(res != MFRC522_T::STATUS_OK)
							{
							connection_serial_transmit(1,(uint8_t*)err_data);
							break;
							}
						resp_len = sizeof(resp);
						res = reader.mfp_read_block(1,resp,&resp_len);
						if(res != MFRC522_T::STATUS_OK)
							{
							connection_serial_transmit(1,(uint8_t*)err_data);
							break;
							}
						connection_serial_transmit(1,resp);

						/*
							res = reader.mfp_following_auth(0x4002,(uint8_t*)mfp_master_key);
							printf("Write block: ");
							uint8_t data[16] = "Hello Cyphrax\r\n";
							res = reader.mfp_write_block(1,data,sizeof(data));
							printf("%i \r\n" ,res);
							*/
						};
					break;
					default:
						{
						connection_serial_transmit(2,(uint8_t*)&mifare_serial.uidByte);
						};
					}
				reader.PICC_HaltA();
				}
			}
		delay(100);
		}
	}

