#include <stm32f0xx.h>

#define RCC_AHBENR_WEIGAND_GPIOEN RCC_AHBENR_GPIOAEN
#define RCC_APB1ENR_WEIGAND_TIMEREN  RCC_APB1ENR_TIM3EN
#define WEIGAND_TIMER_IRQn TIM3_IRQn
#define WEIGAND_GPIO GPIOA
#define WEIGAND_TIMER TIM3
#define WEIGAND_D0_PIN 9 /**< Номер вывода для вывода D0 */
#define WEIGAND_D1_PIN 10 /**< Номер вывода для вывода D1 */


class WEIGAND_SLAVE_T
	{
	public:

		uint8_t data[6];  /**< Данные на отправку */
		const static uint8_t SIZE = 42; /**< Размер данных для отправки */
		/**
		* Должно вызыватся в таймере
		*/
		void timer_handler();
		
    /**
		* Отправляет данные
		* @param Данные
		* Предпологается, что Weigand работает с размером в SIZE, где остаток от деления на 8 является количеством контрольных бит
		*/
		void send(uint8_t* d);

		/**
		* Инициализирует оборудование
		*/
		void init();
    
    /**
    * Конструктор
    */
    WEIGAND_SLAVE_T()
      {
      data_bit_cnt = 0;
      }
			
	private:
		uint32_t data_bit_cnt; /**< Счетчик отправленных бит */
    
    /**
    * Получает бит по номеру с 
    */
    inline bool get_bit(uint8_t b); 
    /**
    * Устанавливает бит
    */
    inline void set_bit(uint8_t b, bool d);
 	};
