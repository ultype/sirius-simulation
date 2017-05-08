#ifndef MODELS_DRIVER_INCLUDE_DRIVER_RPI_SPI_H_
#define MODELS_DRIVER_INCLUDE_DRIVER_RPI_SPI_H_

#include <stddef.h>
#include <stdint.h>

enum {
    SPI_CS0 = 0,
    SPI_CS1,
};

enum {
    SPI_BIT_LSBFIRST = 0,
    SPI_BIT_MSBFIRST,
};

enum {
    SPI_MODE0 = 0,
    SPI_MODE1 = 1,
    SPI_MODE2 = 2,
    SPI_MODE3 = 3,
};

int spi_init(void);
int spi_configure(uint32_t sdev,
                  uint32_t clk,
                  uint32_t bit_order,
                  uint32_t spi_mode);
int spi_transfern(uint32_t sdev, char *buff, size_t size);
void spi_exit(void);

#endif  // MODELS_DRIVER_INCLUDE_DRIVER_RPI_SPI_H_
