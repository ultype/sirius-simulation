#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include "driver/rpi/gpio.h"

#define BCM2708_PERI_BASE 0x3f000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or
// SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *((gpio) + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *((gpio) + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a)      \
    *((gpio) + (((g) / 10))) |= \
        (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET \
    *((gpio) + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR \
    *((gpio) + 10)  // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*((gpio) + 13) & (1 << g))  // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *((gpio) + 37)      // Pull up/pull down
#define GPIO_PULLCLK0 *((gpio) + 38)  // Pull up/pull down clock

// I/O access
static volatile unsigned *gpio = NULL;

void gpio_init(void) {
    int mem_fd;
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }

    /* mmap GPIO */
    gpio = (volatile unsigned *)mmap(
        NULL,                    // Any adddress in our space will do
        BLOCK_SIZE,              // Map length
        PROT_READ | PROT_WRITE,  // Enable reading & writting to mapped memory
        MAP_SHARED,              // Shared with other processes
        mem_fd,                  // File to map
        GPIO_BASE);              // Offset to GPIO peripheral

    close(mem_fd);  // No need to keep mem_fd open after mmap

    if (gpio == MAP_FAILED) {
        printf("mmap error 0x%x\n", (uint32_t)gpio);  // errno also set!
        exit(-1);
    }
}


void gpio_config_input(uint32_t g) {
    INP_GPIO(g);
}

void gpio_config_output(uint32_t g) {
    OUT_GPIO(g);
}

void gpio_set(uint32_t g) {
    GPIO_SET = 1 << g;
}

void gpio_clr(uint32_t g) {
    GPIO_CLR = 1 << g;
}

uint32_t gpio_get(uint32_t g) {
    return GET_GPIO(g);
}
