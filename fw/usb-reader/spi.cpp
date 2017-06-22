#include <stm32f0xx.h>
#include "../spi.hpp"


#define RCC_GPIO_ENA RCC_AHBENR_GPIOAEN
#define GPIO_SPI GPIOA
#define GPIO_ENA GPIOA

#define SPI_SCK_PIN 5
#define SPI_MISO_PIN 6
#define SPI_MOSI_PIN 7
#define SPI_ENA_PIN 4

void SPI_T::ena(uint8_t state)
	{
	if(state)
		{
		GPIO_ENA->ODR &= ~(1<<SPI_ENA_PIN);
		}
	else
		{
		GPIO_ENA->ODR |= (1<<SPI_ENA_PIN);
		}
	}

uint8_t SPI_T::transfer(uint8_t data)
	{
	*(uint8_t *)&(SPI1->DR) = data;
	while (SPI1->SR & SPI_SR_BSY);
	while(!(SPI1->SR & SPI_SR_TXE));
	return SPI1->DR; 	
	}

void SPI_T::init()
	{
	RCC->AHBENR |= RCC_GPIO_ENA;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		
	GPIO_ENA->MODER |= (1<<(SPI_ENA_PIN<<1));
	GPIO_ENA->ODR |= (1<<SPI_ENA_PIN);
	GPIO_SPI->MODER |= (2<<(SPI_SCK_PIN<<1))|(2<<(SPI_MISO_PIN<<1))|(2<<(SPI_MOSI_PIN<<1));

	SPI1->CR1 |= SPI_CR1_BR; 
  SPI1->CR1 &= ~SPI_CR1_CPOL; 
  SPI1->CR1 &= ~SPI_CR1_CPHA; 
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; 
  SPI1->CR1 &= ~SPI_CR1_SSM; 
  SPI1->CR1 &= ~SPI_CR1_SSI; 
  SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_FRXTH ; 
  SPI1->CR1 |= SPI_CR1_MSTR; 
  SPI1->CR1 |= SPI_CR1_SPE; 

	 while (SPI1->SR & SPI_SR_BSY);
	}

