/*
 * init.c
 *
 * Created on: May 10th, 2017
 * Author: 
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include "kdriver.h"
#include "hw.h"

// device supported
static
struct pci_device_id daq_device_ids[] = {
   { PCI_DEVICE(ADVANTECH_VID, DEVID_1737) },
   { PCI_DEVICE(ADVANTECH_VID, DEVID_1739) },
   { 0, }
};
MODULE_DEVICE_TABLE(pci, daq_device_ids);

/************************************************************************
* FILE OPERATIONS
************************************************************************/
static
struct file_operations daq_fops = {
   .owner    = THIS_MODULE,
   .open     = daq_file_open,
   .release  = daq_file_close,
   .mmap     = daq_file_mmap,
   .unlocked_ioctl = daq_file_ioctl,
};

/************************************************************************
* device settings
************************************************************************/
static
void daq_device_load_setting(daq_device_t  *daq_dev)
{
   DEVICE_SHARED *shared = &daq_dev->shared;
   int i;

   // DIO
   for (i = 0; i < DIO_PORT_COUNT_MAX; ++i){
      shared->DioPortDir[i]    = DEF_DIO_DIR;
      shared->DiPortInverse[i] = DEF_DI_INVERSE;
      shared->DoPortState[i]   = DEF_DO_STATE;
   }

   for (i = 0; i < DI_INT_COUNT_MAX; ++i){
      shared->DiintTrigEdge[i] = DEF_DIINT_TRIGEDGE;
      shared->DiintGateCtrl[i] = DEF_DIINT_GATE;
   }
}

/************************************************************************
* sysfs support routines
************************************************************************/
#include <biosysfshelper.h>

// ------------------------------------------------------------------
// Device initialize/de-initailize
// ------------------------------------------------------------------
static
int __devinit daq_device_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
   dev_t         devid;
   daq_device_t  *daq_dev;
   DEVICE_SHARED *shared;
   struct device *sysfs_dev;
   size_t        mem_size;
   int           ret;

#define CHK_RESULT(_ok_, err, err_label)   if (!(_ok_)) { ret = err; goto err_label; }

   if ((ret = pci_enable_device(dev)) != 0) {
      daq_trace((KERN_ERR": pci_enable_device failed\n"));
      return ret;
   }
   pci_set_master(dev);   /* enable bus-master */

   // allocate private data structure
   mem_size = (sizeof(daq_device_t) + PAGE_SIZE - 1) & PAGE_MASK;
   daq_dev  = (daq_device_t*)kzalloc(mem_size, GFP_KERNEL);
   CHK_RESULT(daq_dev, -ENOMEM, err_alloc_dev)

   // initialize the private data in the device
   shared               = &daq_dev->shared;
   shared->Size         = sizeof(*shared);
   shared->DeviceNumber = -1; // unknown
   shared->BusNumber    = dev->bus->number;
   shared->SlotNumber   = PCI_SLOT(dev->devfn);
   shared->IoBase       = dev->resource[2].start & ~1UL;
   shared->IoLength     = dev->resource[2].end - shared->IoBase;
   shared->Irq          = dev->irq;
   shared->InitOnLoad   = DEF_INIT_ON_LOAD;

   switch(id->device)
   {
   default: 
   case DEVID_1737: shared->ProductId = BD_PCI1737U; break;
   case DEVID_1739: shared->ProductId = BD_PCI1739U; break;
   }

   /* request I/O regions */
   CHK_RESULT(\
      request_region(shared->IoBase, shared->IoLength, DEVICE_NAME_FROM_PID(shared->ProductId)),\
      -EBUSY, err_no_res);

   /* request irq */
   ret = request_irq(shared->Irq, (daq_irq_handler_t)daq_irq_handler, IRQF_SHARED, DRIVER_NAME, daq_dev);
   CHK_RESULT(ret == 0, ret, err_irq);


   spin_lock_init(&daq_dev->dev_lock);
   tasklet_init(&daq_dev->dev_tasklet, daq_dev_tasklet_func, (unsigned long)daq_dev);
   INIT_LIST_HEAD(&daq_dev->file_ctx_list);
   daq_dev->file_ctx_pool_size = (mem_size - sizeof(daq_device_t)) / sizeof(daq_file_ctx_t);
   if (daq_dev->file_ctx_pool_size){
      daq_dev->file_ctx_pool = (daq_file_ctx_t *)(daq_dev + 1);
   } else {
      daq_dev->file_ctx_pool = NULL;
   }

#if (TISPACE_CUSTOMIZED == 1)
     atomic_set(&daq_dev->is_wallclock_arrive, 0);
#endif /* TISPACE_CUSTOMIZED */

   /* get device information */
   if (dev->subsystem_device >= DEV_MIN_VER) {
      shared->BoardId = AdxIoInB(shared->IoBase, DR_BID) & 0xf;
   } else {
      shared->BoardId = 0;
   }

   /*Get dynamic device number*/
   devid = daq_devid_alloc();
   CHK_RESULT(devid > (dev_t)0, -ENOSPC, err_no_devid)

   /*register our device into kernel*/
   cdev_init(&daq_dev->cdev, &daq_fops);
   daq_dev->cdev.owner = THIS_MODULE;
   ret = cdev_add(&daq_dev->cdev, devid, 1);
   CHK_RESULT(ret == 0, ret, err_cdev_add)

   /* register our own device in sysfs, and this will cause udev to create corresponding device node */
   sysfs_dev = DAQ_SYSFS_INITIALIZE(devid, daq_dev);
   CHK_RESULT(!IS_ERR(sysfs_dev), PTR_ERR(sysfs_dev), err_sysfs_reg)

   // link the info into the other structures
   pci_set_drvdata(dev, daq_dev);
   SetPageReserved(virt_to_page((unsigned long)daq_dev));

   // initialize the device
   daq_device_load_setting(daq_dev);
   daq_dio_initialize_hardware(daq_dev);

   // Winning horn
   daq_trace((KERN_INFO "Add %s: major:%d, minor:%d\n", DEVICE_NAME_FROM_PID(shared->ProductId), MAJOR(devid), MINOR(devid)));
   daq_trace((KERN_INFO "dev base addr:[0x%x, %d], bid:%d\n", shared->IoBase, shared->IoLength, shared->BoardId));

   return 0;

err_sysfs_reg:
   cdev_del(&daq_dev->cdev);

err_cdev_add:
   daq_devid_free(devid);

err_no_devid:
   free_irq(shared->Irq, daq_dev);

err_irq:
   release_region(shared->IoBase, shared->IoLength);

err_no_res:
   kfree(daq_dev);

err_alloc_dev:
   pci_disable_device(dev);

   daq_trace((KERN_ERR "Add %s failed. error = %d\n", DRIVER_NAME, ret));
   return ret;
}

void daq_device_cleanup(daq_device_t * daq_dev)
{
   // Delete device node under /dev
   cdev_del(&daq_dev->cdev);
   daq_devid_free(daq_dev->cdev.dev);
   device_destroy(daq_class_get(), daq_dev->cdev.dev);

   // Free the device information structure
   ClearPageReserved(virt_to_page((unsigned long)daq_dev));
   kfree(daq_dev);
}

static
void __devexit daq_device_remove(struct pci_dev *dev)
{
   daq_device_t *daq_dev = pci_get_drvdata(dev);
   DEVICE_SHARED *shared = &daq_dev->shared;

   daq_trace((KERN_INFO"Device removed!\n" ));

   release_region(shared->IoBase, shared->IoLength);
   free_irq(shared->Irq, daq_dev);

   pci_set_drvdata(dev, NULL);
   pci_disable_device(dev);
   {
      unsigned long flags;
      spin_lock_irqsave(&daq_dev->dev_lock, flags);
      if (list_empty(&daq_dev->file_ctx_list)){
         spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
         daq_device_cleanup(daq_dev);
      } else {
         daq_dev->remove_pending = 1;
         spin_unlock_irqrestore(&daq_dev->dev_lock, flags);
      }
   }
}

// ------------------------------------------------------------------
// Driver initialize/de-initailize
// ------------------------------------------------------------------
static
struct pci_driver pci_driver = {
   .name      = DRIVER_NAME,
   .probe     = daq_device_probe,
   .remove    = __devexit_p(daq_device_remove),
   .id_table  = daq_device_ids,
};

static
int __init daq_driver_init(void)
{
   return pci_register_driver(&pci_driver);
}

static
void __exit daq_driver_exit(void)
{
   pci_unregister_driver(&pci_driver);
}

module_init(daq_driver_init);
module_exit(daq_driver_exit);

MODULE_LICENSE("GPL");
