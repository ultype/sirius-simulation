/*
 * kdriver.h
 *
 * Created on: October 7, 2012
 * Author: 
 */

#ifndef _KERNEL_MODULE_H_
#define _KERNEL_MODULE_H_

#ifndef CONFIG_PCI
#  error "This driver needs to have PCI support."
#endif

#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <bdaqdef.h>
#include <ioctls.h>
#include <biokernbase.h>
#include <plx905x.h>
#include <i82c54.h>
#include "kshared.h"

typedef struct daq_file_ctx{
   struct list_head   ctx_list;
   struct daq_device  *daq_dev;
   HANDLE             events[KrnlSptedEventCount];
   int                write_access;
   int                busy;
}daq_file_ctx_t;

typedef struct daq_device
{
   DEVICE_SHARED         shared;
   struct cdev           cdev;

   struct tasklet_struct dio_tasklet;
   spinlock_t            dev_lock;
   int                   remove_pending;
   struct list_head      file_ctx_list;
   daq_file_ctx_t        *file_ctx_pool;
   int                   file_ctx_pool_size;
}daq_device_t;

/************************************************************************/
/* Functions                                                            */
/************************************************************************/
//
// init.c
//
void daq_device_cleanup(daq_device_t * daq_dev);

//
// fops.c
//
int  daq_file_open(struct inode *in, struct file *fp);
int  daq_file_close(struct inode *inode, struct file *filp);
int  daq_file_mmap(struct file *filp, struct vm_area_struct *vma);
long daq_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int  daq_device_signal_event(daq_device_t *daq_dev, unsigned index);
int  daq_device_clear_event(daq_device_t *daq_dev, unsigned index);

//
// dio.c
//
void daq_dio_initialize_hw(daq_device_t *daq_dev);
int daq_ioctl_di_read_port(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_write_port(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_read_port(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_diint_set_param(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_start_snap(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_stop_snap(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_write_bit(daq_device_t *daq_dev, unsigned long arg);
//
// isr.c
//
irqreturn_t daq_irq_handler(int irq, void *dev_id);
void daq_dio_tasklet_func(unsigned long arg);

#endif /* _KERNEL_MODULE_H_ */
