#include "weigand.hpp"

void WEIGAND_SLAVE_T::timer_handler()
	{
	if(WEIGAND_TIMER->SR & TIM_SR_CC2IF)
		{
		if(data_bit_cnt < SIZE)
			{
			WEIGAND_GPIO->ODR &= ~(((data[data_bit_cnt >> 3] & (1<<(data_bit_cnt&7))) == (1<<(data_bit_cnt&7))) ? (1<<WEIGAND_D1_PIN) : (1<<WEIGAND_D0_PIN));
			data_bit_cnt++;
			}
		else
			{
			WEIGAND_TIMER->CR1 &= ~TIM_CR1_CEN;
			}
		WEIGAND_TIMER->SR &= ~TIM_SR_CC2IF;
		}


	if(WEIGAND_TIMER->SR & TIM_SR_CC1IF)
		{
		WEIGAND_GPIO->ODR |= (1<<WEIGAND_D0_PIN);
		WEIGAND_GPIO->ODR |= (1<<WEIGAND_D1_PIN);
		WEIGAND_TIMER->SR &= ~TIM_SR_CC1IF;
		}
	}

void WEIGAND_SLAVE_T::init()
	{
	RCC->AHBENR |= RCC_AHBENR_WEIGAND_GPIOEN;
	WEIGAND_GPIO->MODER |= (1 << (WEIGAND_D0_PIN<<1)) | (1 << (WEIGAND_D1_PIN<<1));
	WEIGAND_GPIO->ODR |= (1<<WEIGAND_D0_PIN)|(1<<WEIGAND_D1_PIN);

	/** WEIGAND_TIMER */
	RCC->APB1ENR |= RCC_APB1ENR_WEIGAND_TIMEREN;

	WEIGAND_TIMER->DIER |= TIM_DIER_CC2IE|TIM_DIER_CC1IE;
	WEIGAND_TIMER->ARR = 0xc000;
	WEIGAND_TIMER->PSC = 0;
	WEIGAND_TIMER->CCR2 = 0xc000; //1.024ms
	//WEIGAND_TIMER->CCR1 = 0x1333; //102.4us
	//WEIGAND_TIMER->CCR2 = 0xfffe; //1.356ms
	//WEIGAND_TIMER->CCR1 = 0x2fff; //255.9us
	WEIGAND_TIMER->CCR1 = 0x2000; //170.72us

	NVIC_EnableIRQ(WEIGAND_TIMER_IRQn);
	}

inline bool WEIGAND_SLAVE_T::get_bit(uint8_t b)
	{
	return ((data[b>>3] & (1<<(b&7))) == (1<<b&7));
	}

inline void WEIGAND_SLAVE_T::set_bit(uint8_t b, bool d)
	{
	if(d)
		data[b>>3] |= 1<<(b&7);
	else
		data[b>>3] &= ~(1<<(b&7));
	}

void WEIGAND_SLAVE_T::send(uint8_t* d)
	{
	bool even_parity_bit = 0;
	bool odd_parity_bit = 0;

	if(!(WEIGAND_TIMER->CR1 & TIM_CR1_CEN))
		{
		for(uint8_t i = 0; i < (SIZE-2)>>1 ; i++)
			{
			uint8_t b = d[i>>3];
			b <<= i&7;
			even_parity_bit ^= ((b & 0x80) == 0x80);
			}

		for(uint8_t i = ((SIZE-2)>>1); i <= (SIZE-1)  ; i++)
			{
			uint8_t b = d[i>>3];
			b <<= i&7;
			odd_parity_bit ^= ((b & 0x80) == 0x80);
			}

		set_bit(0,!even_parity_bit);
		set_bit(41,!odd_parity_bit);

		for(uint8_t byte_num = 0; byte_num < 5; byte_num++)
			{
			uint8_t byte_val = d[byte_num];
			for(uint8_t i = 0; i < 8; i++)
					{
					uint8_t bit_num = i + 1 + (byte_num * 8);
					set_bit(bit_num,0);
					bool bit_val = (byte_val & 0x80) == 0x80;
					byte_val <<=1;
					set_bit(bit_num,bit_val);
					}
			}

		data_bit_cnt = 0;
		WEIGAND_TIMER->CR1 |= TIM_CR1_CEN;
		}
	else
		{
		data_bit_cnt = 0;
		}
	}

