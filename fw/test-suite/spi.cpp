#include "stm32f10x.h"
#include "spi.hpp"



void SPI_T::ena(uint8_t state)
{
	if(state)
	{
	GPIO_ResetBits(GPIO_ENA, (1<<SPI_ENA_PIN));
	}
	else
	{
	GPIO_SetBits(GPIO_ENA, (1<<SPI_ENA_PIN));
	}
}

uint8_t SPI_T::transfer(uint8_t data)
{   
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) { ; }
	return SPI_I2S_ReceiveData(SPI2);
}

void SPI_T::init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = (1<<SPI_ENA_PIN);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_ENA, &GPIO_InitStructure);

	ena(0);
	GPIO_InitStructure.GPIO_Pin   = (1<<SPI_SCK_PIN) | (1<<SPI_MOSI_PIN);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_SPI, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = (1<<SPI_MISO_PIN);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIO_SPI, &GPIO_InitStructure);
	SPI_InitTypeDef  SPI_InitStructure;
	
	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 72000kHz/256=281kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_CalculateCRC(SPI2, DISABLE);
	SPI_Cmd(SPI2, ENABLE);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) { ; }
	uint8_t dummyread = SPI_I2S_ReceiveData(SPI2);
}

