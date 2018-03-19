/*******************************************************************************
Copyright (c) 1983-2012 Advantech Co., Ltd.
********************************************************************************
THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY INFORMATION
WHICH IS THE PROPERTY OF ADVANTECH CORP., ANY DISCLOSURE, USE, OR REPRODUCTION,
WITHOUT WRITTEN AUTHORIZATION FROM ADVANTECH CORP., IS STRICTLY PROHIBITED. 

================================================================================
REVISION HISTORY
--------------------------------------------------------------------------------
$Log:  $

--------------------------------------------------------------------------------
$NoKeywords:  $
*/

/******************************************************************************
*
* Windows Example:
*    DIInterrupt.cpp
*
* Example Category:
*    DIO
*
* Description:
*    This example demonstrates how to use DI snap function with a channel interrupt event
*
* Instructions for Running:
*    1. Set the 'deviceDescription' for opening the device. 
*    2. Set the 'startPort' as the first port for Di scanning.
*    3. Set the 'portCount' to decide how many sequential ports to operate Di scanning.

* I/O Connections Overview:
*    Please refer to your hardware reference manual.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "../inc/compatibility.h"
#include "../../../inc/bdaqctrl.h"
using namespace Automation::BDaq;
//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the demo
//-----------------------------------------------------------------------------------
#define  deviceDescription  L"PCI-1737,BID#0"
int32    startPort = 0;
int32    portCount = 1;

struct ioctl_tispace_cmd {
   long long time_tics;
};

#define BDAQ_DIO_MAGIC  'd'
#define IOCTL_DIO_TISPACE_CUSTOMIZED_WAIT_GPIO_INT  _IO(BDAQ_DIO_MAGIC, 15)
#define IOCTL_DIO_TISPACE_CUSTOMIZED_GET_WALLCLOCK_TIME_NS  _IOR(BDAQ_DIO_MAGIC, 16, struct ioctl_tispace_cmd)


inline void waitAnyKey()
{
   do{SLEEP(1);} while(!kbhit());
} 
// This class is used to deal with 'DIInterrupt' Event, we should overwrite the virtual function DiSnapEvent.
class DiSnapHandler : public DiSnapEventListener
{
public:
   virtual void BDAQCALL DiSnapEvent(void * sender, DiSnapEventArgs * args)
   {
      //show snap data.
      printf(" DI Interrupt channel is %d\n",args->SrcNum);
      for ( int32 i = startPort;i < startPort + portCount; ++i )
      {
         printf(" DI port %d status:  0x%X\n\n", i, args->PortData[i-startPort]);
      }
   }
};

int main(int argc, char* argv[])
{
    ErrorCode ret = Success;
    char date_buf[80];
    char currentTime[84] = "";
    static struct timespec ts;
    unsigned int milli;
    struct ioctl_tispace_cmd wall_time;
   // Step 1: Create a 'InstantDiCtrl' for DI function.
   InstantDiCtrl * instantDiCtrl = AdxInstantDiCtrlCreate();

	// Step 2: Set the notification event Handler by which we can known the state of operation effectively.
#if 0
    DiSnapHandler DiSnap;
	instantDiCtrl->addInterruptListener(DiSnap);
#endif
   do
   {
      // Step 3: Select a device by device number or device description and specify the access mode.
      // in this example we use AccessWriteWithReset(default) mode so that we can 
      // fully control the device, including configuring, sampling, etc.
      DeviceInformation devInfo(deviceDescription);
      ret = instantDiCtrl->setSelectedDevice(devInfo);
      CHK_RESULT(ret);

      // Step 4: Set necessary parameters for DI operation, 
      // Note: some of operation of this step is optional(you can do these settings via "Device Configuration" dialog).
      ICollection<DiintChannel>* interruptChans = instantDiCtrl->getDiintChannels();
      // In this demo, we are using the first available one.
		if (interruptChans == NULL)
		{
			printf(" The device doesn't support DI interrupt!\n");
			waitAnyKey();
			return 0;
		}
      interruptChans->getItem(startPort).setEnabled(true);
      printf(" DI channel %d is used to detect interrupt!\n\n", interruptChans->getItem(startPort).getChannel());

      // Step 5: Start DIInterrupt
      ret = instantDiCtrl->SnapStart();
      CHK_RESULT(ret);

      // Step 6: Do anything you are interesting while the device is working.
      printf(" Snap has started, any key to quit !\n");
      do
      {
#if 1
          SLEEP(1);// do something yourself !
          ioctl(3, IOCTL_DIO_TISPACE_CUSTOMIZED_GET_WALLCLOCK_TIME_NS, &wall_time);
          fprintf(stderr, "[%lld us] ioctl received \n", wall_time.time_tics/1000);
#endif
#if 0
        ioctl(3, GPIOEVENT_IOCTL, 0);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec = time(NULL);
        milli = ts.tv_nsec / 1000000;
        strftime(date_buf, (size_t) 20, "%Y/%m/%d,%H:%M:%S", localtime(&ts.tv_sec));
        snprintf(currentTime, sizeof(currentTime), "%s.%03d", date_buf, milli);
        fprintf(stderr, "[%s] ioctl received\n", currentTime);
#endif
      }while (!kbhit());

      // Step 7: Stop DIInterrupt
      ret = instantDiCtrl->SnapStop();
      CHK_RESULT(ret);
   }while(false);
  
	// Step 8: Close device, release any allocated resource.
	instantDiCtrl->Dispose();

	// If something wrong in this execution, print the error code on screen for tracking.
   if(BioFailed(ret))
   {
      printf("Some error occurred. And the last error code is Ox%X.\n", ret);
      waitAnyKey();
   } 
   return 0;
}


