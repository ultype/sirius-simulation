#include "ExternalSourceClock.hh"
#include "trick/message_proto.h"
#include "trick/message_type.h"

#define EXT_CLOCK_TICS_PER_SEC 1000
#define EXT_WALLCLOCK_TICS_RATIO (1000000000LL/EXT_CLOCK_TICS_PER_SEC)

int32_t    startPort = 0;
int32_t    portCount = 1;
/**
@details
-# Calls the base Clock constructor
*/
ExternalSourceClock::ExternalSourceClock() : Clock(EXT_CLOCK_TICS_PER_SEC, "ExternalSourceClock")
{ }

/**
@details
-# This function is empty
*/
ExternalSourceClock::~ExternalSourceClock() { }

/**
@details
-# Set the global "the_clock" pointer to this instance
*/
int ExternalSourceClock::clock_init() {
    ErrorCode ret = Success;

    // Step 1: Create a 'InstantDiCtrl' for DI function.
    instantDiCtrl = AdxInstantDiCtrlCreate();

    // Step 3: Select a device by device number or device description and specify the access mode.
    // in this example we use AccessWriteWithReset(default) mode so that we can
    // fully control the device, including configuring, sampling, etc.
    DeviceInformation devInfo(deviceDescription);
    ret = instantDiCtrl->setSelectedDevice(devInfo);

    // Step 4: Set necessary parameters for DI operation,
    // Note: some of operation of this step is optional(you can do these settings via "Device Configuration" dialog).
    ICollection<DiintChannel>* interruptChans = instantDiCtrl->getDiintChannels();
    // In this demo, we are using the first available one.
    if (interruptChans == NULL)
    {
        printf(" The device doesn't support DI interrupt!\n");
        return 0;
    }
    interruptChans->getItem(startPort).setEnabled(true);
    printf(" DI channel %d is used to detect interrupt!\n\n", interruptChans->getItem(startPort).getChannel());

    // Step 5: Start DIInterrupt
    ret = instantDiCtrl->SnapStart();

    // Step 6: Do anything you are interesting while the device is working.
    printf(" Snap has started, any key to quit !\n");
    set_global_clock();
    return 0;
}

/**
@details
-# Call the system clock_gettime to get the current real time.
-# Return the current real time as a count of microseconds
*/
long long ExternalSourceClock::wall_clock_time() {
    struct ioctl_tispace_cmd wall_time;
    if (ioctl(6, IOCTL_DIO_TISPACE_CUSTOMIZED_GET_WALLCLOCK_TIME_NS, &wall_time) < 0) {
        fprintf(stderr, "[%s:%d] ioctl error \n", __FUNCTION__, __LINE__);
    }
    //  fprintf(stderr, "[%lld us] wall_clock_time \n", wall_time.time_tics/1000);
    /* wall_time.time_tics = nanoseconds*/
    return wall_time.time_tics/EXT_WALLCLOCK_TICS_RATIO;
}

/**
@details
-# This function is empty
*/
int ExternalSourceClock::clock_stop() {
    // Step 7: Stop DIInterrupt
    instantDiCtrl->SnapStop();
    // Step 8: Close device, release any allocated resource.
    instantDiCtrl->Dispose();
    set_global_clock();
    return 0;
}
