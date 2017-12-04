#ifndef __SENDCAN_H__
#define __SENDCAN_H__
extern timer_t timerid;
extern struct itimerspec its;
double get_curr_time(void);
#endif