/*
 * isr.c
 *
 * Created on: October 7, 2012
 * Author: 
 */
#include "kdriver.h"
#include "hw.h"

// -----------------------------------------------------------------------------
// Must hold the lock when call this function.
void daq_dio_tasklet_func(unsigned long arg)
{
   daq_device_t  *daq_dev = (daq_device_t *) arg;
   DEVICE_SHARED *shared  = &daq_dev->shared;
   unsigned long flags;
   __u32         int_state, src_idx;

   // Copy the interrupt state to local cache to avoid impose the interrupt handler.
   spin_lock_irqsave(&daq_dev->dev_lock, flags);
   int_state = shared->DiIntState;
   shared->DiIntState = 0;
   spin_unlock_irqrestore(&daq_dev->dev_lock, flags);

   // check interrupt state
   for (src_idx = 0; int_state; int_state >>= 1, ++src_idx) {
      if (int_state & 0x1) {
          if (shared->DiSnap[src_idx].Count) {
             __u32 i;
             for ( i = 0; i < DIO_PORT_COUNT; ++i ) {
                shared->DiSnap[src_idx].State[i] = AdxIoInB(shared->IoBase, DR_DI_PORTX(i));
             }
          }

          // signal the event if needed.
          if (!shared->IsEvtSignaled[src_idx + KdxDiBegin]) {
             shared->IsEvtSignaled[src_idx + KdxDiBegin] = 1;
             daq_device_signal_event(daq_dev, src_idx + KdxDiBegin);
          }
      }
   }
}

irqreturn_t daq_irq_handler(int irq, void *dev_id)
{
   daq_device_t  *daq_dev = (daq_device_t *) dev_id;
   DEVICE_SHARED *shared  = &daq_dev->shared;

   __u32 int_state = AdxIoInW(shared->IoBase, DR_INT_STA) & DEV_INT_MASK;
   if (!int_state) {
      return IRQ_RETVAL(0);
   }

   spin_lock(&daq_dev->dev_lock);
   shared->DiIntState |= int_state;
   spin_unlock(&daq_dev->dev_lock);

   tasklet_schedule(&daq_dev->dio_tasklet);

   // Clear interrupt
   AdxIoOutW(shared->IoBase, DR_INT_CLR, int_state);

   return IRQ_RETVAL(1);
}

