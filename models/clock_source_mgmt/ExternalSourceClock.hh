#ifndef __EXTERNAL_CLOCK_SOURCE_HH__
#define __EXTERNAL_CLOCK_SOURCE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      TRX Ctrl
LIBRARY DEPENDENCY:
      (
        (ExternalSourceClock.cpp)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <time.h>
#include <sys/ioctl.h>
#include "trick/Clock.hh"
#include "bdaqctrl.h"
#include "compatibility.h"
using namespace Automation::BDaq;

#define  deviceDescription  L"PCI-1737,BID#0"
#define  EXT_CLK_IOCTL_FD_NUM (6)
struct ioctl_tispace_cmd {
   long long time_tics;
};

#define BDAQ_DIO_MAGIC  'd'
#define IOCTL_DIO_TISPACE_CUSTOMIZED_WAIT_GPIO_INT  _IO(BDAQ_DIO_MAGIC, 15)
#define IOCTL_DIO_TISPACE_CUSTOMIZED_GET_WALLCLOCK_TIME_NS  _IOR(BDAQ_DIO_MAGIC, 16, struct ioctl_tispace_cmd)

class ExternalSourceClock : public Trick::Clock {
 public:
        InstantDiCtrl * instantDiCtrl;
        ExternalSourceClock();
        ~ExternalSourceClock();

        /** @copybrief Trick::Clock::clock_init() */
        virtual int clock_init();

        /** @copybrief Trick::Clock::wall_clock_time() */
        virtual long long wall_clock_time();

        /** @copybrief Trick::Clock::clock_stop() */
        virtual int clock_stop();
};

#endif /* __EXTERNAL_CLOCK_SOURCE_HH__ */
