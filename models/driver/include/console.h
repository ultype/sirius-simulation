#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#ifdef __unix__
#include <stdio.h>
#define console_init(x)
#define dbg_printf printf
#else
void console_init(void);
int dbg_printf(const char *format, ...);
#endif

int TestPrintf(void);

#endif
