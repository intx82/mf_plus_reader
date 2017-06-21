
#include <stdint.h>

#ifndef SPI_HPP
#define SPI_HPP

#define GPIO_SPI GPIOB
#define GPIO_ENA GPIOA

#define SPI_SCK_PIN 13
#define SPI_MISO_PIN 14
#define SPI_MOSI_PIN 15
#define SPI_ENA_PIN 8

class SPI_T
{
public:
  uint8_t transfer(uint8_t data);
  void ena(uint8_t state);
  void init();
  void reset(uint8_t state);
};
#endif
