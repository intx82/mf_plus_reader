#include <stm32f0xx.h>

/**
* Задумка проста:
* Используется таймер и прерывание по изменению состояния пина. Таймер зпускаеться по COMPARE1. Т.е по установке порта в 0
* Compare2 используется для нахождения таймаута сигнала.
* При установке вывода в 1, происходит прерывание по порту. Время которое порт был установлен в 0, будет доступно в таймере.
* В зависимости от длительности можно вычеслить, что произошло (Reset, передача 1, передача 0)
*/

#define OW_SLAVE_TIMER TIM3 /**< таймер используемый для отсчета времени */

#define OW_SLAVE_TIMER_IRQn TIM3_IRQn /**< Номер прерывания таймера */
#define OW_SLAVE_EXTI_IRQn EXTI4_15_IRQn /**< Номер прерывания порта */
#define OW_SLAVE_GPIO GPIOA /**< Порт 1w */
#define OW_SLAVE_GPIO_PIN 9 /**< Номер вывода для прерывания порта */
#define OW_SLAVE_TIMER_PIN 10 /**< Номер вывода для управления таймером */
#define OW_SLAVE_RCC_GPIOEN RCC_AHBENR_GPIOAEN /**< Бит для включения порта */
#define OW_SLAVE_RCC_TIMEREN RCC_APB1ENR_TIM3EN /**< Бит для включения таймера */

class OW_SLAVE_T
	{
	public:
		/**
		* Возможные состоянии ведомого устройства
		*/
		enum state_t
			{
			RESET, /**< Сброс устройства */
			BUS_OP, /**< Операция с шиной (прием комманд) */
			USER_OP /**< Пользовательские операции (доступ к памяти, итд) */
			} ;

		/**
		* Команды 1wire
		*/
		enum cmd_t
			{
			NULL_CMD = 0,
			WRITE_SCRATCHPAD = 0x0f, /**< Для эмуляции ds1990, тоже чтение серийного номера, причем NAC вроде отправляет именно эту комманду */
			READ_ROM = 0x33, /**< Чтение серийного номера */
			MATCH_ROM = 0x55, /**< Адресация на основе серийного номера*/
			SKIP_ROM = 0xcc,  /**< Пропуск адресации устройства */
			CONDITIONAL_SEARCH = 0xec,  /**< Параметрический поиск устройства */
			SEARCH_ROM = 0xf0,  /**< Поиск устройства */
			};

		uint8_t cmd; /**< Команда 1wire */
		uint8_t serial_num[8]; /**< Серийный номер устройства */
		bool is_condition;
		/**
		* Конструктор
		*/
		OW_SLAVE_T()
			{
			cmd = (cmd_t)0;
			cmd_bit_cnt = 0;
			data_bit_cnt = 0;
			state = RESET;
			is_condition = enable = false;
			condition_search_state = SEND_NONINV_SERIAL;
			presence_counter = 0;
			}

		/**
		* Функция выполняется для сброса состояния при переполнении таймера (Таймаут)
		* Должна выполнятся в прерывании по перполнению таймера (В нашем случаем TIM3)
		*/
		void timeout_handler();

		/**
		* Функция обработки данных (Прерывания входа, установки входа в 1)
		* Должна выполнятся в прерывании по изменению входа (EXTI4_15_IRQHandler)
		*/
		void pin_interrupt_handler();
		/**
		* Инициализирует оборудование
		*/
		void init();
		/**
		* Вычесляет CRC
		*/
		uint8_t crc(uint8_t *data, uint8_t count);
		/**
		* Функция для установки серийного номера. По окончанию передачи бит enable будет установлен в 0 (Режим эмуляции ds1990)
		*/
		void set_serial(uint8_t *serial, bool is_calc_crc=true);
	private:
		/**
		* Функция установки порта в состояние
		*/
		inline void set_port(bool i);
		/**
		* Функция задержки, время в мкс
		*/
		inline void delay_us(uint16_t d);

		/**
		* Устанавливает состояние порта в 0 на время TW1L (true) TW0L (false)
		*/
		inline void send(bool bit);

		/**
		* Приимает бит. Если время >= TW0L то возвращает false
		* @param Время импульса
		*/
		inline bool recv(uint16_t time);
	///Есть идея отказатся от enable и перенести эти дела в timer->ena и exti->ena 
		bool enable; /**< Указывает включен ли модуль/либа */

		/**
		* Тайминги
		*/
		static const uint16_t TRST = 480; /**< Время RESET импульса */
		static const uint16_t TPDH = 30; /**< Время на которое в presence будет установлена 1*/
		static const uint16_t TPDL = 120; /**< Время на которое в presence будет установлена 0 */
		static const uint16_t TW0L = 45; /**< Время установки шины в 0 для передачи 0 */
		static const uint16_t TW1L = 2; /**< Время установки шины в 0 для передачи 1 */

		static const uint8_t crc88540_table[256]; /**< Таблица для построения CRC */
		static const uint8_t presence_max_count = 2; /**< Максимальное количество опросов presence от nac-а */

		uint8_t cmd_bit_cnt; /**< Счетчик принятых байт команды */
		uint8_t data_bit_cnt; /**< Счетчик принятых байт данных */
		state_t state; /**< Состояние устройства */
		/**
		* Так получилось, что NAC весьма тупое существо, и после первого считывания для проверки карточки он не использует полный цикл опроса серийника,
		*	а просто смотрит - отвечает ли считыватель на presence-ом на reset, если да значит карточка еще в поле.
		* Что бы эту проблему решить, есть --костыль-- счетчик presence_counter, если presence был опрошен более presence_max_count, и не было полного считывания номера (BUS_OP)
		* То 1wire просто отключается.
		*/
		uint8_t presence_counter;

		/**
		* Состояния автомата при поиске устройства
		*/
		enum  condition_search_state_t
			{
			SEND_NONINV_SERIAL, /**< Отправка не инвертированого бита серийного номера */
			SEND_INV_SERIAL, /**< Отправка инвертированого бита серийного номера */
			RECV_MASTER_SERIAL, /**< Прием решения мастера */
			} ;

		condition_search_state_t condition_search_state;
	};

