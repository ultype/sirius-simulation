/*
 * kshared.h
 *
 * Created on: May 10th, 2017
 * Author: 
 */

#ifndef KERNEL_MODULE_SHARED_H_
#define KERNEL_MODULE_SHARED_H_

#include <linux/types.h>


#define ADVANTECH_VID 0x13fe
#define DEVID_1737    0x1737
#define DEVID_1739    0x1739

#define DRIVER_NAME   "bio1737"

#define DEVICE_ID_FROM_PID(pid)   (pid == BD_PCI1737U ? DEVID_1737 : DEVID_1739)
#define DEVICE_NAME_FROM_PID(pid) (pid == BD_PCI1737U ? "PCI-1737" : "PCI-1739")

// ----------------------------------------------------------
// H/W feature and structures.
// ----------------------------------------------------------
#define DIO_PORT_COUNT(prod_id)  (prod_id == BD_PCI1737U ? 3 : 6)
#define DIO_PORT_COUNT_MAX       6  // 1739

#define DIO_CH_COUNT(prod_id)    (DIO_PORT_COUNT(prod_id) * 8)
#define DIO_CH_COUNT_MAX         (DIO_PORT_COUNT_MAX * 8)  

#define DI_INT_COUNT(prod_id)    (prod_id == BD_PCI1737U ? 1 : 2) // CH#16, CH#40(*,PCI-1739 only)
#define DI_INT_COUNT_MAX         2

#define DI_SNAP_COUNT(prod_id)   DI_INT_COUNT(prod_id)
#define DI_SNAP_COUNT_MAX        DI_INT_COUNT_MAX


enum KRNL_EVENT_IDX{
   KdxDevPropChged = 0,
   KdxDiBegin,
   KdxDiintChan16 = KdxDiBegin,
   KdxDiintChan40,
   KdxDiEnd = KdxDiintChan40,
   KrnlSptedEventCount,
};

static inline __u32 kdxofEvent(__u32 eventType)
{
   __u32 kdx;
   switch ( eventType )
   {
   case EvtPropertyChanged: kdx = KdxDevPropChged; break;
   case EvtDiintChannel016: kdx = KdxDiintChan16;  break;
   case EvtDiintChannel040: kdx = KdxDiintChan40;  break;
   default: kdx = -1; break;
   }
   return kdx;
}


// -----------------------------------------------------
// default values
// -----------------------------------------------------
#define DEF_INIT_ON_LOAD        1

// DIO default values
#define DEF_DIO_DIR             0x00
#define DEF_DI_INVERSE          0
#define DEF_DO_STATE            0
#define DEF_DIINT_TRIGEDGE      RisingEdge
#define DEF_DIINT_GATE          0

// ----------------------------------------------------------
// Device private data
// ----------------------------------------------------------
typedef struct _DI_SNAP
{
   __u8 Start;
   __u8 Count;
   __u8 State[DIO_PORT_COUNT_MAX];
} DI_SNAP;

typedef struct _DEVICE_SHARED
{
   __u32   Size;           // Size of the structure
   __u32   ProductId;      // Device Type
   __u32   DeviceNumber;   // Zero-based device number

   // HW Information
   __u32   BoardId;        // Board dip switch number for the device
   __u32   BusNumber;      // PCI Bus number
   __u32   SlotNumber;     // PCI Slot number
   __u32   IoBase;
   __u32   IoLength;
   __u32   Irq;
   __u32   InitOnLoad;

   // --------------------------------------------------------
   __u8    DioPortDir[DIO_PORT_COUNT_MAX]; 
   __u8    DiPortInverse[DIO_PORT_COUNT_MAX];
   __u8    DoPortState[DIO_PORT_COUNT_MAX];
   
   __u8    DiintTrigEdge[DI_INT_COUNT_MAX];
   __u8    DiintGateCtrl[DI_INT_COUNT_MAX];
   DI_SNAP DiSnap[DI_SNAP_COUNT_MAX];

   // ---------------------------------------------------------
   __u32   IsEvtSignaled[KrnlSptedEventCount];

} DEVICE_SHARED;

#endif /* KERNEL_MODULE_SHARED_H_ */
