/*
 * fops.c
 *
 * Created on: May 10th, 2017
 * Author: 
 */
#include "kdriver.h"
#include "hw.h"

static
daq_file_ctx_t * daq_file_alloc_context(daq_device_t *daq_dev)
{
   daq_file_ctx_t *ctx = NULL;
   unsigned long  flags;
   int i;

   if (likely(daq_dev->file_ctx_pool_size)){
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      for (i = 0; i < daq_dev->file_ctx_pool_size; ++i){
         if (!daq_dev->file_ctx_pool[i].busy){
            ctx = &daq_dev->file_ctx_pool[i];
            ctx->busy = 1;
            break;
         }
      }
      spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
   }

   if (ctx == NULL){
      ctx = (daq_file_ctx_t*)kzalloc(sizeof(daq_file_ctx_t), GFP_KERNEL);
   }

   if (ctx){
      INIT_LIST_HEAD(&ctx->ctx_list);
      ctx->daq_dev = daq_dev;
   }

   return ctx;
}

static
void daq_file_free_context(daq_file_ctx_t *ctx)
{
   daq_device_t  *daq_dev = ctx->daq_dev;
   unsigned long flags;

   if (likely(daq_dev->file_ctx_pool_size
       && daq_dev->file_ctx_pool <= ctx
       && ctx < daq_dev->file_ctx_pool + daq_dev->file_ctx_pool_size)){
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      ctx->busy = 0;
      spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
   } else {
      kfree(ctx);
   }
}

static
int daq_ioctl_dev_get_desc(daq_device_t *daq_dev, unsigned long arg)
{
   char desc[64];
   int  len;

   len = snprintf(desc, sizeof(desc), "%s,BID#%d", DEVICE_NAME_FROM_PID(daq_dev->shared.ProductId), daq_dev->shared.BoardId);
   if (unlikely(copy_to_user((void *)arg, desc, len + 1))){
     return -EFAULT;
   }

   return 0;
}

static
int daq_ioctl_dev_get_location(daq_device_t *daq_dev, unsigned long arg)
{
   char loc[64];
   int  len;

   len = snprintf(loc, sizeof(64), "pci-%d-%d", daq_dev->shared.BusNumber, daq_dev->shared.SlotNumber);
   if (unlikely(copy_to_user((void *)arg, loc, len + 1))){
      return -EFAULT;
   }

   return 0;
}

static
int daq_ioctl_dev_reg_event(daq_device_t *daq_dev, daq_file_ctx_t *ctx, unsigned long arg)
{
   unsigned long flags;
   unsigned      kdx;
   uint32        event_id;
   HANDLE        event;

   if (unlikely(get_user(event_id, &((USER_EVENT_INFO*)arg)->EventId))){
      return -EFAULT;
   }

   kdx = kdxofEvent(event_id);
   if (likely(kdx < KrnlSptedEventCount)){
      event = ctx->events[kdx];
      if (event == NULL){
         event = daq_event_create();

         spin_lock_irqsave(&daq_dev->dev_lock, flags);
         if (ctx->events[kdx] == NULL){
            ctx->events[kdx] = event;
         } else {
            daq_event_close(event);
            event = ctx->events[kdx];
         }
         spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
      }
      put_user(event, &((USER_EVENT_INFO*)arg)->Handle);
      return event != NULL ? 0 : -ENOMEM;
   }

   return -EINVAL;
}

static
int daq_ioctl_dev_unreg_event(daq_device_t *daq_dev, daq_file_ctx_t *ctx, unsigned long arg)
{
   unsigned long flags;
   unsigned      kdx = kdxofEvent(arg);

   if (likely(kdx < KrnlSptedEventCount)){
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      if (ctx->events[kdx] != NULL) {
         daq_event_close(ctx->events[kdx]);
         ctx->events[kdx] = NULL;
      }
      spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
   }

   return 0;
}

static
int daq_ioctl_dev_dbg_reg_in(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED *shared = &daq_dev->shared;
   DBG_REG_IO    xbuf;
   __u32         value;
   int           ret;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   if (xbuf.Length > sizeof(value)
      || (xbuf.Addr - shared->IoBase) > shared->IoLength){
      return -EINVAL;
   }

   switch(xbuf.Length)
   {
   case 1:
      value = AdxIoInB(xbuf.Addr, 0);
      ret   = put_user(value, (__u8 *)xbuf.Data);
      break;
   case 2:
      value = AdxIoInW(xbuf.Addr, 0);
      ret   = put_user(value, (__u16 *)xbuf.Data);
      break;
   case 4:
      value = AdxIoInD(xbuf.Addr, 0);
      ret   = put_user(value, (__u32 *)xbuf.Data);
      break;
   default:
      ret = -EINVAL;
      break;
   }

   return ret;
}

static
int daq_ioctl_dev_dbg_reg_out(daq_device_t *daq_dev, unsigned long arg)
{
   DEVICE_SHARED *shared = &daq_dev->shared;
   DBG_REG_IO    xbuf;
   __u32         value;
   int           ret;

   if (unlikely(copy_from_user(&xbuf, (void *)arg, sizeof(xbuf)))){
      return -EFAULT;
   }

   if (xbuf.Length > sizeof(value)
      || (xbuf.Addr - shared->IoBase) > shared->IoLength){
      return -EINVAL;
   }

   switch(xbuf.Length)
   {
   case 1:
      if (!(ret = get_user(value, (__u8 *)xbuf.Data))){
         AdxIoOutB(xbuf.Addr, 0, value);
      }
      break;
   case 2:
      if (!(ret = get_user(value, (__u16 *)xbuf.Data))){
         AdxIoOutW(xbuf.Addr, 0, value);
      }
      break;
   case 4:
      if (!(ret = get_user(value, (__u32 *)xbuf.Data))){
         AdxIoOutD(xbuf.Addr, 0, value);
      }
      break;
   default:
      ret = -EINVAL;
      break;
   }

   return ret;
}

//-------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------
int daq_file_open(struct inode *in, struct file *fp)
{
   daq_device_t   *daq_dev = container_of(in->i_cdev, daq_device_t, cdev);
   daq_file_ctx_t *ctx;
   unsigned long  flags;
   int            first_user, fmode_chk_ret = 0;

   if (unlikely(daq_dev->remove_pending)){
      return -ENODEV;
   }

   ctx = daq_file_alloc_context(daq_dev);
   if (unlikely(ctx == NULL)){
      return -ENOMEM;
   }
   ctx->write_access = fp->f_mode & FMODE_WRITE;

   spin_lock_irqsave(&daq_dev->dev_lock, flags);

   first_user = list_empty(&daq_dev->file_ctx_list);
   if (ctx->write_access){
      daq_file_ctx_t *curr;
      list_for_each_entry(curr, &daq_dev->file_ctx_list, ctx_list){
         if (curr->write_access) {
            fmode_chk_ret = -EPERM;
            break;
         }
      }
   }

   if (!fmode_chk_ret){
      list_add(&ctx->ctx_list, &daq_dev->file_ctx_list);
      fp->private_data = ctx;
   }

   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);

   if (fmode_chk_ret){
      daq_file_free_context(ctx);
   }

   return fmode_chk_ret;
}

int daq_file_close(struct inode *inode, struct file *filp)
{
   daq_file_ctx_t *ctx     = filp->private_data;
   daq_device_t   *daq_dev = ctx->daq_dev;
   unsigned long  flags;
   int            i, last_user;

   if (ctx->write_access) {
      AdxIoOutB(daq_dev->shared.IoBase, DR_INT_CS, DEV_INT_MASK);
   }

   spin_lock_irqsave(&daq_dev->dev_lock, flags);
   list_del(&ctx->ctx_list);
   last_user = list_empty(&daq_dev->file_ctx_list);
   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);

   for (i = 0; i < KrnlSptedEventCount; ++i){
      if (ctx->events[i]){
         daq_event_close(ctx->events[i]);
         ctx->events[i] = NULL;
      }
   }

   daq_file_free_context(ctx);

   if (last_user){
      if (daq_dev->remove_pending){
         daq_device_cleanup(daq_dev);
      }
   }

   return 0;
}

int daq_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
   if (vma->vm_pgoff == 0){
      daq_device_t * daq_dev = ((daq_file_ctx_t *)filp->private_data)->daq_dev;
      if (vma->vm_end - vma->vm_start > PAGE_SIZE){
         return -EIO;
      }

      return remap_pfn_range(vma, vma->vm_start,
               virt_to_phys((void *)daq_dev) >> PAGE_SHIFT,
               vma->vm_end - vma->vm_start, vma->vm_page_prot);
   }

   return -EIO;
}

long daq_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   daq_file_ctx_t *ctx     = filp->private_data;
   daq_device_t   *daq_dev = ctx->daq_dev;
   int            ret = 0;

   switch(cmd)
   {
   //******************************************************************************************
   // IOCTL for device operation                                                              *
   //******************************************************************************************
   case IOCTL_DEVICE_GET_DESC:
      ret = daq_ioctl_dev_get_desc(daq_dev, arg);
      break;
   case IOCTL_DEVICE_GET_LOCATION_INFO:
      ret = daq_ioctl_dev_get_location(daq_dev, arg);
      break;
   case IOCTL_DEVICE_REGISTER_USER_EVENT:
      ret = daq_ioctl_dev_reg_event(daq_dev, ctx, arg);
      break;
   case IOCTL_DEVICE_UNREGISTER_USER_EVENT:
      ret = daq_ioctl_dev_unreg_event(daq_dev, ctx, arg);
      break;
   case IOCTL_DEVICE_NOTIFY_PROP_CHGED:
      ret = daq_device_signal_event(daq_dev, KdxDevPropChged);
      break;
   case IOCTL_DEVICE_DBG_REG_IN:
      ret = daq_ioctl_dev_dbg_reg_in(daq_dev, arg);
      break;
   case IOCTL_DEVICE_DBG_REG_OUT:
      ret = daq_ioctl_dev_dbg_reg_out(daq_dev, arg);
      break;
   //******************************************************************************************
   // IOCTL for DIO operation                                                                 *
   //******************************************************************************************
   case IOCTL_DIO_SET_PORT_DIR:
      ret = daq_ioctl_dio_set_port_dir(daq_dev, arg);
      break;
   case IOCTL_DIO_READ_DI_PORTS:
      ret = daq_ioctl_di_read_port(daq_dev, arg);
      break;
   case IOCTL_DIO_WRITE_DO_PORTS:
      ret = daq_ioctl_do_write_port(daq_dev, arg);
      break;
   case IOCTL_DIO_WRITE_DO_BIT:
      ret = daq_ioctl_do_write_bit(daq_dev, arg);
      break;
   case IOCTL_DIO_READ_DO_PORTS:
      ret = daq_ioctl_do_read_port(daq_dev, arg);
      break;
   case IOCTL_DIO_SET_DIINT_CFG:
      ret = daq_ioctl_diint_set_param(daq_dev, arg);
      break;
   case IOCTL_DIO_START_DI_SNAP:
      ret = daq_ioctl_di_start_snap(daq_dev, arg);
      break;
   case IOCTL_DIO_STOP_DI_SNAP:
      ret = daq_ioctl_di_stop_snap(daq_dev, arg);
      break;
#if (TISPACE_CUSTOMIZED == 1)
   case IOCTL_DIO_TISPACE_CUSTOMIZED_WAIT_GPIO_INT:
        ret = daq_ioctl_di_wait_gpio_int(daq_dev, arg);
      break;
    case IOCTL_DIO_TISPACE_CUSTOMIZED_GET_WALLCLOCK_TIME_NS:
        ret = daq_ioctl_di_get_wallclock_time(daq_dev, arg);
        break;
#endif /* TISPACE_CUSTOMIZED */
   default:
      ret = -ENOTTY;
      break;
   }

   return ret;
}

int daq_device_signal_event(daq_device_t *daq_dev, unsigned kdx)
{
   daq_file_ctx_t *curr;
   unsigned long  flags;

   if (kdx < KrnlSptedEventCount){
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      list_for_each_entry(curr, &daq_dev->file_ctx_list, ctx_list){
         if (curr->events[kdx] != NULL){
            daq_event_set(curr->events[kdx]);
         }
      }
      spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
      return 0;
   }

   return -EINVAL;
}

int daq_device_clear_event(daq_device_t *daq_dev, unsigned kdx)
{
   daq_file_ctx_t *curr;
   unsigned long  flags;

   if (kdx < KrnlSptedEventCount){
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      list_for_each_entry(curr, &daq_dev->file_ctx_list, ctx_list){
         if (curr->events[kdx] != NULL){
            daq_event_reset(curr->events[kdx]);
         }
      }
      spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
      return 0;
   }

   return -EINVAL;
}


