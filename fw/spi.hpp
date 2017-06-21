
#include <stdint.h>

#ifndef SPI_HPP
#define SPI_HPP


class SPI_T
{
public:
  uint8_t transfer(uint8_t data);
  void ena(uint8_t state);
  void init();
  void reset(uint8_t state);
};
#endif
