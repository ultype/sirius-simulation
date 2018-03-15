/*
 * hw.h
 *
 * Created on: October 7, 2012
 * Author: 
 */

#ifndef _KERNEL_MODULE_HW_H_
#define _KERNEL_MODULE_HW_H_

#define DR_DI_BASE         0
#define DR_DI_PORTX(x)     (DR_DI_BASE + x)

#define DR_DO_BASE         0
#define DR_DO_PORTX(x)     (DR_DO_BASE + x)

#define DR_BID             0x4

#define TRIG_EDGE_RISING   0
#define TRIG_EDGE_FALLING  1
#define DR_INT_TRIGEDGE    0xc

#define DEV_INT_MASK       0xF  // for fast detect the interrupt status
#define DR_INT_EN          0x8
#define DR_INT_STA         0x10
#define DR_INT_CLR         DR_INT_STA

#endif /* _KERNEL_MODULE_HW_H_ */
