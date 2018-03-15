/*
 * isr.c
 *
 * Created on: May 10th, 2017
 * Author: 
 */
#include "kdriver.h"
#include "hw.h"

extern __u32 g_dio_port_offset[];

static inline __u8 * daq_isr_snap_di_port(DEVICE_SHARED *shared, __u8 buffer[DIO_PORT_COUNT_MAX])
{
   __u8     *ptr = buffer;
   unsigned PORT_MAX = DIO_PORT_COUNT(shared->ProductId);
   unsigned i;

   for (i = 0; i < PORT_MAX; ++i, ++ptr) {
      *ptr = AdxIoInB(shared->IoBase, g_dio_port_offset[i]);
   }
   return buffer;
}

// -----------------------------------------------------------------------------
// device interrupt tasklet
void daq_dev_tasklet_func(unsigned long arg)
{
   daq_device_t  *daq_dev = (daq_device_t *) arg;
   DEVICE_SHARED *shared  = &daq_dev->shared;
   __u8          *snapped = NULL;
   DEV_INT_CS_R  int_state;   
   unsigned long flags;

   // Copy the interrupt state to local cache to avoid impose the interrupt handler.
   spin_lock_irqsave(&daq_dev->dev_lock, flags);
   int_state.xval = (__u8)daq_dev->dev_int_state;
   daq_dev->dev_int_state = 0;
   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);

   // check interrupt state
   if (int_state.GRP0_FLAG) {
      if (shared->DiSnap[0].Count) {
         snapped = daq_isr_snap_di_port(shared, shared->DiSnap[0].State);
      }

      if (!shared->IsEvtSignaled[KdxDiintChan16]) {
         shared->IsEvtSignaled[KdxDiintChan16] = 1;
#if (TISPACE_CUSTOMIZED == 0)
         daq_device_signal_event(daq_dev, KdxDiintChan16);
#endif  /* TISPACE_CUSTOMIZED */
      }
   }

   if (int_state.GRP1_FLAG) {
      if (shared->DiSnap[1].Count) {
         if (snapped) {
            memcpy(shared->DiSnap[1].State, snapped, DIO_PORT_COUNT_MAX);
         } else {
            snapped = daq_isr_snap_di_port(shared, shared->DiSnap[1].State);
         }
      }

      if (!shared->IsEvtSignaled[KdxDiintChan40]) {
         shared->IsEvtSignaled[KdxDiintChan40] = 1;
         daq_device_signal_event(daq_dev, KdxDiintChan40);
      }
   }
}

irqreturn_t daq_irq_handler(int irq, void *dev_id)
{
   daq_device_t *daq_dev = (daq_device_t *) dev_id;
   __u32        int_state;
   
   int_state = AdxIoInB(daq_dev->shared.IoBase, DR_INT_CS);
   if (!(int_state  & DEV_INT_MASK)) {
      return IRQ_RETVAL(0);
   }

   spin_lock(&daq_dev->dev_lock);
   daq_dev->dev_int_state |= int_state;
   spin_unlock(&daq_dev->dev_lock);
#if (TISPACE_CUSTOMIZED == 1)
    if (wait_task != NULL) {
        //  printk("<0>""[%d:%s] wake up process !!\n", __LINE__, __FUNCTION__);
        local_irq_disable();
        wake_up_process(wait_task);
    }
#else
   tasklet_schedule(&daq_dev->dev_tasklet);
#endif /* TISPACE_CUSTOMIZED */
   // Clear interrupt
   AdxIoOutB(daq_dev->shared.IoBase, DR_INT_CS, int_state);
   daq_trace((KERN_INFO"device int: %d\n", int_state));
   return IRQ_RETVAL(1);
}

