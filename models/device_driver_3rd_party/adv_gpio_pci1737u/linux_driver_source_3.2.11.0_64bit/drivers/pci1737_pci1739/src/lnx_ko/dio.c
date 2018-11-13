/*
 * dio.c
 *
 * Created on: May 10th, 2017
 * Author: 
 */
#include "kdriver.h"
#include "hw.h"

#if (TISPACE_CUSTOMIZED == 1)
static atomic_t user_flag = ATOMIC_INIT(1);
struct task_struct *wait_task = NULL;
#endif /* TISPACE_CUSTOMIZED */


// DIO port Data register
__u32 g_dio_port_offset[] = {
   DR_GRPx_PORTy(0, 0), DR_GRPx_PORTy(0, 1), DR_GRPx_PORTy(0, 2),
   DR_GRPx_PORTy(1, 0), DR_GRPx_PORTy(1, 1), DR_GRPx_PORTy(1, 2),
};

void daq_dio_set_port_dir(daq_device_t *daq_dev)
{
   __u8 const *dirs = daq_dev->shared.DioPortDir;
   GRP_CS_R ctl;
   uint32 i;

   for (i = 0; i < DIO_PORT_COUNT(daq_dev->shared.ProductId) / PORT_PER_GROUP; ++i)
   {
      ctl.PA      = *dirs++ == 0 ? PORTDIR_IN : PORTDIR_OUT;
      ctl.PB      = *dirs++ == 0 ? PORTDIR_IN : PORTDIR_OUT;
      ctl.PC_LOW  = (*dirs & 0x0F) == 0 ? PORTDIR_IN : PORTDIR_OUT;
      ctl.PC_HIGH = (*dirs & 0xF0) == 0 ? PORTDIR_IN : PORTDIR_OUT;
      ++dirs;
      
      // Does the direction changed ?
      if (ctl.xval != AdxIoInB(daq_dev->shared.IoBase, DR_GRPx_CS(i))) {
         AdxIoOutB(daq_dev->shared.IoBase, DR_GRPx_CS(i), ctl.xval);
      }
   }
}

//-------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------
void daq_dio_initialize_hardware(daq_device_t *daq_dev)
{
   unsigned i;

   daq_dio_set_port_dir(daq_dev);

   if (daq_dev->shared.InitOnLoad){
      for (i = 0; i < DIO_PORT_COUNT(daq_dev->shared.ProductId); ++i) {
         AdxIoOutB(daq_dev->shared.IoBase, g_dio_port_offset[i], daq_dev->shared.DoPortState[i]);
      }
   }
}

int daq_ioctl_dio_set_port_dir(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED   *shared = &daq_dev->shared;
   DIO_SET_PORT_DIR xbuf;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))) {
      return -EFAULT;
   }
   if (xbuf.PortStart + xbuf.PortCount > DIO_PORT_COUNT(shared->ProductId)) {
      return -EINVAL;
   }

   if (unlikely(copy_from_user(shared->DioPortDir + xbuf.PortStart, xbuf.Dirs, xbuf.PortCount))) {
      return -EFAULT;
   }

   daq_dio_set_port_dir(daq_dev);
   return 0;
}

int daq_ioctl_di_read_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u32 PORT_MAX = DIO_PORT_COUNT(daq_dev->shared.ProductId);
   __u32 i, port;
   __u8  data[DIO_PORT_COUNT_MAX];

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   xbuf.PortStart %= PORT_MAX;
   xbuf.PortCount  = x_min(PORT_MAX, xbuf.PortCount);
   for (i = 0; i < xbuf.PortCount; ++i){
      port     = (xbuf.PortStart + i) % PORT_MAX;
      data[i]  = AdxIoInB(daq_dev->shared.IoBase, g_dio_port_offset[port]);
      data[i] ^= daq_dev->shared.DiPortInverse[port];
   }

   if (unlikely(copy_to_user(xbuf.Data, data, xbuf.PortCount))){
      return -EFAULT;
   }

   return 0;
}

int daq_ioctl_do_write_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u32 PORT_MAX = DIO_PORT_COUNT(daq_dev->shared.ProductId);
   __u32 i, port;
   __u8  data[DIO_PORT_COUNT_MAX];

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }
   if (xbuf.PortStart >= PORT_MAX || xbuf.PortCount > PORT_MAX) {
      return -EINVAL;
   }

   if (unlikely(copy_from_user(data, (void *)xbuf.Data, xbuf.PortCount))){
      return -EFAULT;
   }

   for (i = 0; i < xbuf.PortCount; ++i){
      port = (xbuf.PortStart + i) % PORT_MAX;
      daq_dev->shared.DoPortState[port] = data[i];
      AdxIoOutB(daq_dev->shared.IoBase, g_dio_port_offset[port], data[i]);
   }

   return 0;
}

int daq_ioctl_do_write_bit(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_BIT xbuf;
   __u8       *state;
   unsigned long flags;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }
   if (xbuf.Port >= DIO_PORT_COUNT(daq_dev->shared.ProductId)) {
      return -EINVAL;
   }

   state = &daq_dev->shared.DoPortState[xbuf.Port];
   spin_lock_irqsave(&daq_dev->dev_lock, flags);
      *state &= ~(1 << xbuf.Bit);
      *state |= (!!xbuf.Data) << xbuf.Bit;
      AdxIoOutB(daq_dev->shared.IoBase, g_dio_port_offset[xbuf.Port], *state);
   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
   
   return 0;
}

int daq_ioctl_do_read_port(daq_device_t *daq_dev, unsigned long arg)
{
   DIO_RW_PORTS xbuf;
   __u32 PORT_MAX = DIO_PORT_COUNT(daq_dev->shared.ProductId);
   __u32 i, port;
   __u8  data[DIO_PORT_COUNT_MAX];

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   xbuf.PortStart %= PORT_MAX;
   xbuf.PortCount  = x_min(PORT_MAX, xbuf.PortCount);
   for (i = 0; i < xbuf.PortCount; ++i){
      port    = (xbuf.PortStart + i) % PORT_MAX;
      data[i] = AdxIoInB(daq_dev->shared.IoBase, g_dio_port_offset[port]);
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

   if (xbuf.SrcStart + xbuf.SrcCount > DI_INT_COUNT_MAX) {
      return -EINVAL;
   }

   switch(xbuf.SetWhich)
   {
   case DIINT_SET_TRIGEDGE: dest = shared->DiintTrigEdge; break;
   case DIINT_SET_GATECTRL: dest = shared->DiintGateCtrl; break;
   default: 
      return -EINVAL;
   }
   
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

   event_kdx = kdxofEvent(xbuf.EventId);
   src_idx   = event_kdx - KdxDiBegin;
   if (src_idx >= DI_SNAP_COUNT(shared->ProductId)) {
      return -EINVAL;
   }

   daq_device_clear_event(daq_dev, event_kdx);
   shared->IsEvtSignaled[event_kdx] = 0;
   shared->DiSnap[src_idx].Start = (__u8)xbuf.PortStart;
   shared->DiSnap[src_idx].Count = (__u8)xbuf.PortCount;

   {
      DEV_INT_CS_R int_ctl;
      int_ctl.xval = AdxIoInB(shared->IoBase, DR_INT_CS) & (~DEV_INT_MASK);

      if (src_idx == 0) {
         if (int_ctl.GRP0_MODE != INT_DISABLED) {
            return -EBUSY;
         }
         int_ctl.GRP0_MODE = shared->DiintGateCtrl[0] ? INT_SRC_PC0_GATE : INT_SRC_PC0;
         int_ctl.GRP0_EDGE = shared->DiintTrigEdge[0] == RisingEdge ? TRIG_EDGE_RISING : TRIG_EDGE_FALLING;
      } else {
         if (int_ctl.GRP1_MODE != INT_DISABLED) {
            return -EBUSY;
         }
         int_ctl.GRP1_MODE = shared->DiintGateCtrl[1] ? INT_SRC_PC0_GATE : INT_SRC_PC0;
         int_ctl.GRP1_EDGE = shared->DiintTrigEdge[1] == RisingEdge ? TRIG_EDGE_RISING : TRIG_EDGE_FALLING;
      }

      AdxIoOutB(shared->IoBase, DR_INT_CS, int_ctl.xval);
   }

   return 0;
}

int daq_ioctl_di_stop_snap(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED *shared = &daq_dev->shared;
   unsigned      event_kdx, src_idx;

   event_kdx = kdxofEvent((__u32)arg);
   src_idx   = event_kdx - KdxDiBegin;
   if (src_idx >= DI_SNAP_COUNT(shared->ProductId)) {
      return -EINVAL;
   }

   {
      // disable the DI interrupt of the channel
      DEV_INT_CS_R int_ctl;
      int_ctl.xval = AdxIoInB(shared->IoBase, DR_INT_CS) & (~DEV_INT_MASK);

      if (src_idx == 0) {
         int_ctl.GRP0_MODE = INT_DISABLED;
      } else {
         int_ctl.GRP0_MODE = INT_DISABLED;
      }

      AdxIoOutB(shared->IoBase, DR_INT_CS, int_ctl.xval);
   }

   return 0;
}

int daq_ioctl_di_wait_gpio_int(daq_device_t *daq_dev, unsigned long arg)
{
    /* yield as possible */
    while (!atomic_dec_and_test(&user_flag))
        schedule();
    wait_task = current;
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule();
    __set_current_state(TASK_RUNNING);
    wait_task = NULL;
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
    //  printk("<0>""ioctl %d pps %lld\n",daq_dev->pps_cnt , cmd.time_tics);
    if (unlikely(copy_to_user((int __user *)arg, &cmd, sizeof(struct ioctl_tispace_cmd)))){
      return -EFAULT;
    }
    return 0;
}
