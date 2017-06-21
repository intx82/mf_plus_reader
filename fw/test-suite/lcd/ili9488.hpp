#ifndef ILI9488_H
#define ILI9488_H
#include <stdio.h>
#include <stdint.h>
#include "ili9488_regs.h"
#include "stm32f10x.h"

/**
* Настройка порта для данных
*/
#define LCD_IO_DATA GPIOA /**< Порт для данных */
#define LCD_IO_DATA_WRITE LCD_IO_DATA->ODR /**< Регистр на запись  */
#define LCD_IO_DATA_READ  LCD_IO_DATA->IDR /**< Регистра на чтение */
#define LCD_IO_DATA_DIR 	LCD_IO_DATA->CRL /**< Регистр направления */

#define LCD_IO_CTRL GPIOB
#define LCD_IO_CTRL_WRITE LCD_IO_CTRL->ODR
#define LCD_IO_CTRL_DIRL 	LCD_IO_CTRL->CRL
#define LCD_IO_CTRL_DIRH 	LCD_IO_CTRL->CRH

/**
* Класс LCD
*/
class LCD_T
    {
    public:
	
        const static uint16_t WIDTH = 320; /**< Размер экрана от 0 */
        const static uint16_t HEIGHT = 480;
        const static uint8_t IO_CTRL_RD_bit = 8; /**< Номер вывода отвечающий за ногу RD дисплея */
        const static uint8_t IO_CTRL_WR_bit = 9; /**< Номер вывода отвечающий за ногу WR дисплея */
        const static uint8_t IO_CTRL_RS_bit = 1; /**< Номер вывода отвечающий за ногу RS дисплея */
        const static uint8_t IO_CTRL_CS_bit = 5; /**< Номер вывода отвечающий за ногу CS дисплея */
        const static uint8_t IO_CTRL_RST_bit = 0; /**< Номер вывода отвечающий за ногу RST дисплея */
        const static uint32_t IO_DATA_DIR_WRITE_MASK = 0x33333333; /**< Маска регистра CRL для записи данных */
        const static uint32_t IO_DATA_DIR_READ_MASK = 0x44444444; /**<  Макска регистра CRL для чтения данных */
    
		/**
		* Возможные ошибки при работе со шрифтом
		*/
        enum font_err_t {FONT_OK=0,FONT_FS_ERR,FONT_NO_SYMB,FONT_OUT_OF_DISPLAY} ;

		/**
		* Инициализация дисплея
		*/
        void init(void);
		/**
		* инициализация портов дисплея
		*/
        void io_init(void);
		/**
		* Установка параметров вывода данных на дисплей
		*/
        void set_stdout_params(uint16_t x, uint16_t y,uint32_t bgcolor, uint32_t textcolor);
		/**
		* Запись байта комманды в дисплей 
		* @param комманда
		*/ 
        void write_cmd_byte(uint8_t cmd);
		/**
		* Запись байта данных в дисплей 
		* @param данные
		*/ 
        void write_data_byte(uint8_t data);
		/**
		* Чтение байта данных из дисплея
		* @return Данные
		*/ 
        uint8_t read_data_byte(void);
		/**
		* Рисует квадрат на экране
		*/
        void show_rect(uint16_t start_x,uint16_t end_x,uint16_t start_y, uint16_t end_y,uint32_t color);
				
				void line(uint16_t start_x,uint16_t end_x,uint16_t start_y, uint16_t end_y,uint32_t color,uint8_t line_width=1);
		/**
		* Рисует содержимое памяти
		*/
        void show_mem(uint16_t x,uint16_t y, uint8_t width, uint8_t heigth, uint8_t* mem);
		
        void set_area(uint16_t start_x,uint16_t end_x, uint16_t start_y, uint16_t end_y);
        /**
		* Очищает дисплей
		*/
		void clear_screen(void);
		/**
		* Показывает содержимое памяти закодированное через RLE
		*/
        void show_rle_mem(uint16_t x,uint16_t y, uint8_t width, uint8_t heigth, unsigned char *img);
		/**
		* Устанавливает пиксель в цвет
		*/
        void show_pixel(uint32_t color);

		/**
		* Заглушка для отображения буквы
		*/
        void putchar(uint16_t x, uint16_t y, uint8_t chr,uint32_t bg_color,uint32_t text_color);

		/**
		* Установка вывода CS
		*/
        void cs_set(void);
		/**
		* Сброс вывода CS
		*/
        void cs_reset(void);
		/**
		* Переворачивает данные
		*/
        uint8_t reverse (uint8_t b);
		/**
		* Задержка
		*/
        void delay(volatile uint32_t d);
				
					private:
					
					//Автоматически созданная таблица шрифта
				static const uint8_t font[256][5]; 
    };

#endif
