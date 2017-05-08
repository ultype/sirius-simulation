#ifndef MODELS_DRIVER_INCLUDE_DRIVER_CONSOLE_H_
#define MODELS_DRIVER_INCLUDE_DRIVER_CONSOLE_H_

#ifdef __unix__
#include <stdio.h>
#define console_init(x)
#define dbg_printf printf
#else
void console_init(void);
int dbg_printf(const char *format, ...);
#endif

int TestPrintf(void);

#endif  // MODELS_DRIVER_INCLUDE_DRIVER_CONSOLE_H_
