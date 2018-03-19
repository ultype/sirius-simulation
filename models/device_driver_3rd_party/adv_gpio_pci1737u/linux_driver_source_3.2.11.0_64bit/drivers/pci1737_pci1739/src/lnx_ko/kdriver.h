/*
 * kdriver.h
 *
 * Created on: May 10th, 2017
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
#include "kshared.h"

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>

#define TISPACE_CUSTOMIZED 1
#if (TISPACE_CUSTOMIZED == 1)
extern struct task_struct *wait_task;
#endif /* TISPACE_CUSTOMIZED */

typedef struct daq_file_ctx {
   struct list_head   ctx_list;
   struct daq_device  *daq_dev;
   HANDLE             events[KrnlSptedEventCount];
   int                write_access;
   int                busy;
} daq_file_ctx_t;

typedef struct daq_device
{
   DEVICE_SHARED         shared;
   struct cdev           cdev;

   spinlock_t            dev_lock;
   int                   remove_pending;
   struct list_head      file_ctx_list;
   daq_file_ctx_t        *file_ctx_pool;
   int                   file_ctx_pool_size;

   struct tasklet_struct dev_tasklet;
   __u32                 dev_int_state;
#if (TISPACE_CUSTOMIZED == 1)
   ktime_t curr_beg_pps_tics;
   ktime_t curr_end_pps_tics;
   ktime_t prev_end_pps_tics;
   int first_pps;
#endif /* TISPACE_CUSTOMIZED */
} daq_device_t;

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

void daq_dio_initialize_hardware(daq_device_t *daq_dev);
void daq_dio_set_port_dir(daq_device_t *daq_dev);

int daq_ioctl_dio_set_port_dir(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_read_port(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_write_port(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_write_bit(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_do_read_port(daq_device_t *daq_dev, unsigned long arg);

int daq_ioctl_diint_set_param(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_start_snap(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_stop_snap(daq_device_t *daq_dev, unsigned long arg);

int daq_ioctl_di_wait_gpio_int(daq_device_t *daq_dev, unsigned long arg);
int daq_ioctl_di_get_wallclock_time(daq_device_t *daq_dev, unsigned long arg);
//
// isr.c
//
irqreturn_t daq_irq_handler(int irq, void *dev_id);
void daq_dev_tasklet_func(unsigned long arg);

#endif /* _KERNEL_MODULE_H_ */
