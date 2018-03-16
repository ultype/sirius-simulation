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

#include "trick/Clock.hh"

class ExternalSourceClock : public Trick::Clock {
 public:
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
