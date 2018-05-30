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
*    PatternMatchInterrupt.cpp
*
* Example Category:
*    DIO
*
* Description:
*    This example demonstrates how to use DI snap function with a pattern match interrupt event
*
* Instructions for Running:
*    1. Set the 'deviceDescription' for opening the device. 
*    2. Set the 'startPort' as the first port for Do outputting.
*    3. Set the 'portCount' to decide how many sequential ports to operate Do outputting.
*    4. Set pattern match channels and values for supported ports in system device manager configuration.
*
* I/O Connections Overview:
*    Please refer to your hardware reference manual.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "../inc/compatibility.h"
#include "../../../inc/bdaqctrl.h"
using namespace Automation::BDaq;
//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the demo
//-----------------------------------------------------------------------------------
#define  deviceDescription  L"DemoDevice,BID#0"
int32    startPort = 0; 
int32    portCount = 1; 
//uint8  pmEnableChans = 0x01;

inline void waitAnyKey()
{
   do{SLEEP(1);} while(!kbhit());
} 
// This class is used to deal with 'PatternMath' Event, we should overwrite the virtual function DiSnapEvent.
class PatternMatchHandler : public DiSnapEventListener
{
public:
   virtual void BDAQCALL DiSnapEvent(void * sender, DiSnapEventArgs * args)
   {
      unsigned char bufferForSnap[MAX_DIO_PORT_COUNT];
      memcpy(bufferForSnap,args->PortData,MAX_DIO_PORT_COUNT);
      //show snap data.
      printf(" Pattern Match Interrupt port is %d\n",args->SrcNum);
      for ( int32 i = startPort;i < startPort + portCount; ++i )
      {
         printf("DI port %d status is:  0x%X\n\n", i, bufferForSnap[i-startPort]);
      }
   }
};


int main(int argc, char* argv[])
{
   ErrorCode ret = Success;
   // Step 1: Create a 'InstantDiCtrl' for DI function.
   InstantDiCtrl * instantDiCtrl = AdxInstantDiCtrlCreate();

	// Step 2: Set the notification event Handler by which we can known the state of operation effectively.
	PatternMatchHandler patternMath;
	instantDiCtrl->addPatternMatchListener(patternMath);

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
	  // Using diPmintPorts to get all the channels which support DI Pattern Match Interrupt function
      ICollection<DiPmintPort>* diPmintPorts = instantDiCtrl->getDiPmintPorts();
      if(diPmintPorts == NULL)
      {
         printf(" The device doesn't support DI pattern match interrupt!\n any key to quit...");
			waitAnyKey();
         return 0;
      }
      printf(" DI port %d is used to detect pattern match interrupt!\n\n", diPmintPorts->getItem(0).getPort());
      // In this demo, we do not set these parameters and just using the values the user had inputted in 'device configuration' dialog.
	  // Using 'pmEnableChans' mask to choose which channels to be enabled DI Pattern Match Interrupt.
      //diPmintPorts->getItem(0).setMask(pmEnableChans);

	  // In this demo, we do not set these parameters and just using the values the user had inputted in 'device configuration' dialog.
	  // Using 'pmValue' to set which port value to be triggered DI Pattern Match Interrupt.
	  //diPmintPorts->getItem(0).setPattern(pmValue);

      // Step 5: Start PatternMatchInterrupt
      ret = instantDiCtrl->SnapStart();
      CHK_RESULT(ret);

      // Step 6: Do anything you are interesting while the device is working.
      printf(" Snap has started, any key to quit !\n");
      do
      {
         SLEEP(1);// do something yourself !
      }while (!kbhit());

      // Step 7: Stop PatternMatchInterrupt
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

