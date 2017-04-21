#include "driver/rpi/spi.h"
#include "driver/console.h"
#include <bcm2835.h>

uint32_t bcm2835_cs[] = {BCM2835_SPI_CS0, BCM2835_SPI_CS1};
uint32_t bcm2835_bit_order[] = {BCM2835_SPI_BIT_ORDER_LSBFIRST,
                                BCM2835_SPI_BIT_ORDER_MSBFIRST};
uint32_t bcm2835_spi_mode[] = {
    BCM2835_SPI_MODE0,  // CPOL = 0, CPHA = 0
    BCM2835_SPI_MODE1,  // CPOL = 0, CPHA = 1
    BCM2835_SPI_MODE2,  // CPOL = 1, CPHA = 0
    BCM2835_SPI_MODE3   // CPOL = 1, CPHA = 1
};

int spi_configure(uint32_t sdev,
                  uint32_t clk,
                  uint32_t bit_order,
                  uint32_t spi_mode)
{
    uint32_t clk_divider;
    uint32_t cs = sdev; /* In Rpi3, use CS to select device.  */

    if (cs > 2 || bit_order > 2 || spi_mode > 4) {
        dbg_printf(
            "Invalid SPI Configuration CS=%d, bit_order = %d"
            "spi_mode = %d\n",
            cs, bit_order, spi_mode);
        return -1;
    }


    switch (clk) {
        case 200000000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_2;
            break;
        case 100000000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_4;
            break;
        case 50000000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_8;
            break;
        case 25000000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_16;
            break;
        case 12500000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_32;
            break;
        case 6250000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_64;
            break;
        case 3125000:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_128;
            break;
        case 1562500:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_256;
            break;
        case 781250:
            clk_divider = BCM2835_SPI_CLOCK_DIVIDER_512;
            break;
        default:
            dbg_printf("Invalid clk = %u\n", clk);
            return -1;
    }


    bcm2835_spi_setBitOrder(bcm2835_bit_order[bit_order]);
    bcm2835_spi_setDataMode(bcm2835_spi_mode[spi_mode]);
    bcm2835_spi_setClockDivider(clk_divider);
    bcm2835_spi_chipSelect(bcm2835_cs[cs]);
    bcm2835_spi_setChipSelectPolarity(bcm2835_cs[cs], LOW);

    return 0;
}

int spi_init(void)
{
    if (!bcm2835_init()) {
        dbg_printf("bcm2835_init failed\n");
        return -1;
    }

    if (!bcm2835_spi_begin()) {
        dbg_printf("bcm2835_spi_begin failedg. Are you running as root??\n");
        return -1;
    }

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

    return 0;
}

int spi_transfern(uint32_t sdev, char *buff, size_t size)
{
    /* In RPI3, sdev is ignored in this function because
     * SPI device is selected by CS which should be
     * configuted before transmission.
     */

    bcm2835_spi_transfern(buff, size);
    return 0;
}

void spi_exit(void)
{
    bcm2835_spi_end();
    bcm2835_close();
}
