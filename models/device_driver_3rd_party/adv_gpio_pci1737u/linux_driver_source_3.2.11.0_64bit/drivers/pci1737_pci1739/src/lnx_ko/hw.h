/*
 * hw.h
 *
 * Created on: May 10th, 2017
 * Author: 
 */

#ifndef _KERNEL_MODULE_HW_H_
#define _KERNEL_MODULE_HW_H_

#define PORT_PER_GROUP       3
#define DR_GRPx_BASE(x)      ((x) * 4)
#define DR_GRPx_PORTy(x, y)  (DR_GRPx_BASE(x) + y)

// DIO port group configuration register
#define PORTDIR_OUT   0
#define PORTDIR_IN    1
typedef union _GRP_CS_R {
   __u8 xval;
   struct{
      __u8 PC_LOW   : 1;  // Port C lower nibble
      __u8 PB       : 1;  // Port B
      __u8 Reserved : 1;
      __u8 PC_HIGH  : 1;  // Port C higher nibble
      __u8 PA       : 1;  // Port A
      __u8 Reserved2: 3;
   };
} GRP_CS_R;

#define DR_GRPx_CS(x)      (DR_GRPx_BASE(x) + 3)

// interrupt control & status register
#define INT_DISABLED      0
#define INT_SRC_PC0       1
#define INT_SRC_PC0_GATE  2
#define TRIG_EDGE_RISING  1  // rising edge. note: this is the device specified value.
#define TRIG_EDGE_FALLING 0  // falling edge. note: this is the device specified value.
typedef union _DEV_INT_CS_R {
   __u8 xval;
   struct {
      __u8 GRP0_MODE : 2;  // Group interrupt mode. see INT_xxx definition.
      __u8 GRP0_EDGE : 1;  // Group interrupt trigger edge. see TRIG_EDGE_xxx definition.
      __u8 GRP0_FLAG : 1;  // Group 0 interrupt status
      __u8 GRP1_MODE : 2;  // same as GRP0_MODE.
      __u8 GRP1_EDGE : 1;  // same as GRP0_EDGE.
      __u8 GRP1_FLAG : 1;  // same as GRP0_FLAG.
   };
} DEV_INT_CS_R;

#define DEV_INT_MASK  0x88  // for fast detect the interrupt status
#define DR_INT_CS     0x8

// For HW ver. >= 0xA100
#define DEV_MIN_VER   0xA100
#define DR_BID        0x09

#endif /* _KERNEL_MODULE_HW_H_ */
