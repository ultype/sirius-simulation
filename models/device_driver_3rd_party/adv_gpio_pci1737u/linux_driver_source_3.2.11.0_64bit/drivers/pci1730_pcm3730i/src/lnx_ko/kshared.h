/*
 * kshared.h
 *
 * Created on: October 7, 2012
 * Author: 
 */

#ifndef KERNEL_MODULE_SHARED_H_
#define KERNEL_MODULE_SHARED_H_

#include <linux/types.h>


#define ADVANTECH_VID 0x13fe
#define DEVICE_ID     0x1730
#define DEVICE_PID    BD_PCI1730
#define DEVICE_NAME   "PCI-1730"
#define DRIVER_NAME   "bio1730"

// ----------------------------------------------------------
// H/W feature and structures.
// ----------------------------------------------------------
#define DIO_PORT_COUNT		   4 // 2 isolated DI, 2 TTL DI, 2 isolated DO, 2 TTL DO
#define DIO_CHL_COUNT         (DIO_PORT_COUNT * 8)
#define DI_INT_SRC_COUNT      4  // CH#[0, 1, 16, 17]
#define DI_SNAP_SRC_COUNT     DI_INT_SRC_COUNT


enum KRNL_EVENT_IDX{
   KdxDevPropChged = 0,
   KdxDiBegin,
   KdxDiintChan0 = KdxDiBegin,
   KdxDiintChan1,
   KdxDiintChan16,
   KdxDiintChan17,
   KdxDiEnd = KdxDiintChan17,
   KrnlSptedEventCount,
};

static inline __u32 GetEventKIndex(__u32 eventType)
{
   __u32 kdx;
   switch ( eventType )
   {
   case EvtPropertyChanged: kdx = KdxDevPropChged; break;
   case EvtDiintChannel000: kdx = KdxDiintChan0;   break;
   case EvtDiintChannel001: kdx = KdxDiintChan1;   break;
   case EvtDiintChannel016: kdx = KdxDiintChan16;  break;
   case EvtDiintChannel017: kdx = KdxDiintChan17;  break;
   default: kdx = -1; break;
   }
   return kdx;
}


// -----------------------------------------------------
// default values
// -----------------------------------------------------
#define DEF_INIT_ON_LOAD        1

// DIO default values
#define DEF_DO_STATE            0
#define DEF_DIINT_TRIGEDGE      RisingEdge


// ----------------------------------------------------------
// Device private data
// ----------------------------------------------------------
typedef struct _DI_SNAP
{
   __u8 Start;
   __u8 Count;
   __u8 State[DIO_PORT_COUNT];
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
   __u8    DoPortState[DIO_PORT_COUNT];
   __u8    DiintTrigEdge[DI_INT_SRC_COUNT];
   DI_SNAP DiSnap[DI_SNAP_SRC_COUNT];

   // ---------------------------------------------------------
   __u32   DiIntState;
   __u32   IsEvtSignaled[KrnlSptedEventCount];

} DEVICE_SHARED;

#endif /* KERNEL_MODULE_SHARED_H_ */
