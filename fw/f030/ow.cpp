#include "ow.hpp"

const uint8_t OW_SLAVE_T::crc88540_table[256] =
	{
	0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
	35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
	190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
	70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
	219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
	101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
	140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
	17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
	50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
	202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
	87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
	};


void OW_SLAVE_T::timeout_handler()
	{
	if(OW_SLAVE_TIMER->SR & TIM_SR_CC2IF)
		{
		state = RESET;
		OW_SLAVE_TIMER->CNT = 0;
		OW_SLAVE_TIMER->SR &= ~TIM_SR_CC2IF;
		}
	}

void OW_SLAVE_T::pin_interrupt_handler()
	{
	if(OW_SLAVE_GPIO->IDR & (1<<OW_SLAVE_GPIO_PIN))
		{
		if(enable)
			{
			uint16_t impulse_time = OW_SLAVE_TIMER->CNT/48;
			OW_SLAVE_TIMER->CNT = 0;
			switch(state)
				{
				case RESET:
					{
					if(impulse_time > TRST)
						{
						delay_us(TPDH);
						set_port(0);
						delay_us(TPDL);
						set_port(1);
						cmd_bit_cnt = 0;
						data_bit_cnt = 0;
						cmd = 0;
						state = BUS_OP;
						condition_search_state = SEND_NONINV_SERIAL;
						presence_counter++;
						if(presence_counter >= presence_max_count)
							{
							presence_counter = 0;
							enable = false;
							}

						OW_SLAVE_TIMER->CNT = 0;
						}
					};
				break;

				case BUS_OP:
					{
					presence_counter = 0;
					if(cmd_bit_cnt >= 8)
						{
						switch(cmd)
							{
							/**
							* Просто отправляем 64 бита серийника
							*/
							case WRITE_SCRATCHPAD:
							case READ_ROM:
								{
								send(serial_num[data_bit_cnt>>3] & (1<<(data_bit_cnt&0x7)));
								data_bit_cnt++;
								if(data_bit_cnt >= 64)
									{
									state = USER_OP;
									enable = false;
									}
								};
							break;

							/**
							* Если принятый бит не равен биту серийника переходим в ресет
							*/
							case MATCH_ROM:
								{
								bool bit = serial_num[data_bit_cnt>>3] & (1<<(data_bit_cnt&0x07));
								data_bit_cnt++;
								if(recv(impulse_time) != bit)
									{
									state = RESET;
									}
								else if(data_bit_cnt >= 64)
									{
									cmd_bit_cnt = 0;
									data_bit_cnt = 0;
									state = USER_OP;
									}
								};
							break;

							/**
							* Отправляем бит серийника, потом инверсию бита серийника, шина выравнивается колизией.
							* Ждем ответ от мастера, если мастера бит и бит серийника совпали идем дальше, иначе сброс
							*
							* Если условие не выполнено, не реагируем на conditional search, иначе сразу же переходим в search_rom
							*/
							case CONDITIONAL_SEARCH:
								{
								if(!is_condition)
									{
									state = RESET;
									break;
									}
								};

							case SEARCH_ROM:
								{
								bool bit = serial_num[data_bit_cnt>>3] & (1<<(data_bit_cnt&0x07));
								switch(condition_search_state)
									{
									case SEND_NONINV_SERIAL:
										{
										send(bit);
										condition_search_state = SEND_INV_SERIAL;
										}
									break;
									case SEND_INV_SERIAL:
										{
										send(!bit);
										condition_search_state = RECV_MASTER_SERIAL;
										}
									break;
									case RECV_MASTER_SERIAL:
										{
										condition_search_state = SEND_NONINV_SERIAL;
										if(recv(impulse_time) != bit)
											{
											state = RESET;
											}
										else
											{
											data_bit_cnt++;
											if(data_bit_cnt >= 64)
												{
												cmd_bit_cnt = 0;
												data_bit_cnt = 0;
												state = USER_OP;
												}
											}
										};
									break;
									default:
										{
										state = RESET;
										condition_search_state = SEND_INV_SERIAL;
										}
									}
								};
							break;

							case SKIP_ROM:
								{
								state = USER_OP;
								};
							break;


							default:
								{
								state = RESET;
								};
							break;
							}
						}
					else
						{
						/**
						* Если время импульса больше чем время установки 0, то это 0.
						* Время передачи 1 - от 1 до 15 мкс (TW1L)
						* Время передачи 0 - от 60 до 120 мкс. (TW0L)
						* Тут есть нюанс!
						*/
						cmd <<=1;
						cmd |= recv(impulse_time);
						cmd_bit_cnt++;
						}
					};
				break;
				case USER_OP:
					{
					presence_counter = 0;
					state = RESET;
					};
				break;
				default:
					{
					state = RESET;
					}
				}
			}
		}
	else
		{
		OW_SLAVE_TIMER->CNT = 0;
		}
	EXTI->PR |= 1<<OW_SLAVE_GPIO_PIN;
	}

inline void OW_SLAVE_T::set_port(bool i)
	{
	if(i)
		{
		OW_SLAVE_GPIO->MODER &= ~(1<<(OW_SLAVE_GPIO_PIN<<1));
		OW_SLAVE_GPIO->ODR |= (1<<OW_SLAVE_GPIO_PIN);
		}
	else
		{
		OW_SLAVE_GPIO->MODER |= (1<<(OW_SLAVE_GPIO_PIN<<1));
		OW_SLAVE_GPIO->ODR &= ~(1<<OW_SLAVE_GPIO_PIN);
		}
	}

void OW_SLAVE_T::init()
	{
	/**< GPIO */
	RCC->AHBENR |= OW_SLAVE_RCC_GPIOEN;
		
	OW_SLAVE_GPIO->MODER |= 2 << (OW_SLAVE_TIMER_PIN<<1) ;
	OW_SLAVE_GPIO->ODR &= ~(1<<OW_SLAVE_GPIO_PIN);
	
	set_port(1);
	/** TIMER */
	RCC->APB1ENR |= OW_SLAVE_RCC_TIMEREN;
	/**
		* Еще один костыль в номере вывода
	*/
	OW_SLAVE_TIMER->CCMR2 |= TIM_CCMR2_CC3S_0;
	OW_SLAVE_TIMER->CCER |= TIM_CCER_CC3P|TIM_CCER_CC2E;
	OW_SLAVE_TIMER->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_TS_2 | TIM_SMCR_TS_0;
	OW_SLAVE_TIMER->DIER |= TIM_DIER_CC2IE;
	OW_SLAVE_TIMER->CCR2 = 0xf000;

	/** EXTI */
	/**
	* Костыль, я знаю
	*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PB;
	EXTI->IMR |= 1<<OW_SLAVE_GPIO_PIN;
	EXTI->RTSR |= 1<<OW_SLAVE_GPIO_PIN;
	EXTI->FTSR |= 1<<OW_SLAVE_GPIO_PIN;
	/** IRQ */
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_EnableIRQ(OW_SLAVE_TIMER_IRQn);
	OW_SLAVE_TIMER->CR1 |= TIM_CR1_CEN;
	}

inline void OW_SLAVE_T::delay_us(uint16_t d)
	{
	uint32_t old_time = OW_SLAVE_TIMER->CNT/48;
	while((OW_SLAVE_TIMER->CNT/48) < (old_time+d));
	}

inline void OW_SLAVE_T::send(bool bit)
	{
	set_port(0);
	delay_us(bit?TW1L:TW0L);
	set_port(1);
	}

inline bool OW_SLAVE_T::recv(uint16_t impulse_time)
	{
	return (impulse_time > TW0L);
	}

uint8_t OW_SLAVE_T::crc(uint8_t *data, uint8_t count)
	{
	unsigned char result=0;
	while(count--)
		{
		result = crc88540_table[result ^ *data++];
		}
	return result;
	}

void OW_SLAVE_T::set_serial(uint8_t *serial, bool is_calc_crc)
	{
	if(!enable)
		{
		serial_num[7] = (is_calc_crc) ? crc(serial,7):serial[7];
		for(uint8_t i  = 0; i <7; i++)
			{
			serial_num[i] = serial[i];
			}
		enable = true;
		}
	}


