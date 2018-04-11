/*
 * dio.c
 *
 * Created on: October 7, 2012
 * Author: 
 */
#include "kdriver.h"
#include "hw.h"

#if (TISPACE_CUSTOMIZED == 1)
static atomic_t user_flag = ATOMIC_INIT(1);
struct task_struct *wait_task = NULL;
#endif /* TISPACE_CUSTOMIZED */
//-------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------
void daq_dio_initialize_hw(daq_device_t *daq_dev)
{
   if (daq_dev->shared.InitOnLoad){
      int i;
      for (i = 0; i < DIO_PORT_COUNT; ++i) {
         AdxIoOutB(daq_dev->shared.IoBase, DR_DO_PORTX(i), daq_dev->shared.DoPortState[i]);
      }
   }
}

int daq_ioctl_di_read_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u8         data[DIO_PORT_COUNT];
   __u32        i, port;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   xbuf.PortStart %= DIO_PORT_COUNT;
   xbuf.PortCount  = x_min((unsigned)DIO_PORT_COUNT, xbuf.PortCount);
   for (i = 0; i < xbuf.PortCount; ++i){
      port = (xbuf.PortStart + i) % DIO_PORT_COUNT;
      data[i] = AdxIoInB(daq_dev->shared.IoBase, DR_DI_PORTX(port));
   }

   if (unlikely(copy_to_user(xbuf.Data, data, xbuf.PortCount))){
      return -EFAULT;
   }

   return 0;
}

int daq_ioctl_do_write_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u8         data[DIO_PORT_COUNT];
   unsigned     i, port;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   xbuf.PortStart %= DIO_PORT_COUNT;
   xbuf.PortCount  = x_min((unsigned)DIO_PORT_COUNT, xbuf.PortCount);
   if (unlikely(copy_from_user(data, (void *)xbuf.Data, xbuf.PortCount))){
      return -EFAULT;
   }

   xbuf.PortStart %= DIO_PORT_COUNT;
   xbuf.PortCount  = x_min((unsigned)DIO_PORT_COUNT, xbuf.PortCount);
   for (i = 0; i < xbuf.PortCount; ++i){
      port = (xbuf.PortStart + i) % DIO_PORT_COUNT;
      AdxIoOutB(daq_dev->shared.IoBase, DR_DO_PORTX(port), data[i]);
      daq_dev->shared.DoPortState[port] = data[i];
   }

   return 0;
}

int daq_ioctl_do_write_bit(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_BIT xbuf;
   __u8       data, status;
   unsigned   port, bit;
   unsigned long flags;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   port = xbuf.Port % DIO_PORT_COUNT;
   bit  = xbuf.Bit;
   data = xbuf.Data;

   spin_lock_irqsave(&daq_dev->dev_lock, flags);
   status = daq_dev->shared.DoPortState[port];
   status = ((data & 0x1) << bit) | (~(1 << bit) & status);
   AdxIoOutB(daq_dev->shared.IoBase, DR_DO_PORTX(port), status);
   daq_dev->shared.DoPortState[port] = status;
   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
   
   return 0;
}

int daq_ioctl_do_read_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u8         data[DIO_PORT_COUNT];
   unsigned     i, port;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   xbuf.PortStart %= DIO_PORT_COUNT;
   xbuf.PortCount  = x_min((unsigned)DIO_PORT_COUNT, xbuf.PortCount);
   for (i = 0; i < xbuf.PortCount; ++i){
      port = (xbuf.PortStart + i) % DIO_PORT_COUNT;
      data[i] = daq_dev->shared.DoPortState[port];
   }

   if (unlikely(copy_to_user(xbuf.Data, data, xbuf.PortCount))){
      return -EFAULT;
   }

   return 0;
}

int daq_ioctl_diint_set_param(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED     *shared = &daq_dev->shared;
   DIO_SET_DIINT_CFG xbuf;
   __u8              *dest = NULL;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))) {
      return -EFAULT;
   }

   if (xbuf.SrcStart + xbuf.SrcCount > DI_INT_SRC_COUNT) {
      return -EINVAL;
   }

   if (xbuf.SetWhich != DIINT_SET_TRIGEDGE) {
      return -EINVAL;
   }
   dest = shared->DiintTrigEdge;
   dest += xbuf.SrcStart;

   if (unlikely(copy_from_user(dest, xbuf.Buffer, xbuf.SrcCount))) {
      return -EFAULT;
   }

   return 0;
}

int daq_ioctl_di_start_snap(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED     *shared = &daq_dev->shared;
   DIO_START_DI_SNAP xbuf;
   unsigned          event_kdx, src_idx;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))) {
      return -EFAULT;
   }

   event_kdx = GetEventKIndex(xbuf.EventId);
   src_idx   = event_kdx - KdxDiBegin;
   if (src_idx >= DI_SNAP_SRC_COUNT) {
      return -EINVAL;
   }

   daq_device_clear_event(daq_dev, event_kdx);
   daq_dev->shared.IsEvtSignaled[event_kdx] = 0;
   daq_dev->shared.DiSnap[src_idx].Start = (__u8)xbuf.PortStart;
   daq_dev->shared.DiSnap[src_idx].Count = (__u8)xbuf.PortCount;

   {
      int reg;

      // edge setting
      reg  = AdxIoInB(shared->IoBase, DR_INT_TRIGEDGE);
      reg &= ~(1 << src_idx);
      if (shared->DiintTrigEdge[src_idx] == RisingEdge) {
         reg |= TRIG_EDGE_RISING << src_idx;
      } else {
         reg |= TRIG_EDGE_FALLING << src_idx;
      }
      AdxIoOutB(shared->IoBase, DR_INT_TRIGEDGE, reg);

      // enable the DI interrupt of the channel
      reg  = AdxIoInB( shared->IoBase, DR_INT_EN);
      reg |= 1 << src_idx;
      AdxIoOutB(shared->IoBase, DR_INT_EN, reg);
   }

   return 0;
}

int daq_ioctl_di_stop_snap(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED *shared = &daq_dev->shared;
   unsigned      event_kdx, src_idx;

   event_kdx = GetEventKIndex((__u32)arg);
   src_idx   = event_kdx - KdxDiBegin;
   if (src_idx >= DI_SNAP_SRC_COUNT) {
      return -EINVAL;
   }

   {
      // disable the DI interrupt of the channel
      int reg;

      reg  = AdxIoInB(shared->IoBase, DR_INT_EN);
      reg &= ~(1 << src_idx);
      AdxIoOutB(shared->IoBase, DR_INT_EN, reg);
   }

   return 0;
}
#if (TISPACE_CUSTOMIZED == 1)
int daq_ioctl_di_wait_gpio_int(daq_device_t *daq_dev, unsigned long arg)
{
    /* yield as possible */
    while (!atomic_dec_and_test(&user_flag))
        schedule();

    wait_task = current;
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule();
    __set_current_state(TASK_RUNNING);
    local_irq_enable();
    atomic_set(&user_flag, 1);

   return 0;
}

int daq_ioctl_di_get_wallclock_time(daq_device_t *daq_dev, unsigned long arg)
{
    struct ioctl_tispace_cmd cmd;
    ktime_t curr_ptics;
    long long m,n;

    atomic_set(&daq_dev->is_wallclock_arrive, 1);
    curr_ptics = ktime_get();
    m = curr_ptics - daq_dev->prev_pps_tics;
    n = curr_ptics - daq_dev->curr_pps_tics;
    cmd.time_tics = (1000000000LL * n)/(m-n) + daq_dev->curr_ideal_tics;
    //  printk("<0>""ioctl %d pps %lld, %lld\n",daq_dev->pps_cnt , cmd.time_tics);
    if (unlikely(copy_to_user((int __user *)arg, &cmd, sizeof(struct ioctl_tispace_cmd)))){
      return -EFAULT;
    }
    return 0;
}
#endif /* TISPACE_CUSTOMIZED */