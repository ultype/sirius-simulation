#ifndef __GPIO_H__
#define __GPIO_H__
#include <stdint.h>

void gpio_init(void);
void gpio_config_input(uint32_t g);
void gpio_config_output(uint32_t g);
void gpio_set(uint32_t g);
void gpio_clr(uint32_t g);
uint32_t gpio_get(uint32_t g);
#endif
