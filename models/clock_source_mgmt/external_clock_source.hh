#ifndef __EXTERNAL_CLOCK_SOURCE_HH__
#define __EXTERNAL_CLOCK_SOURCE_HH__

#ifdef __linux
#include <time.h>
#endif

#include "trick/Clock.hh"

namespace Trick {
class ExternalSourceClock : public Clock {
 public:
        ExternalSourceClock();
        ~ExternalSourceClock();

        /** @copybrief Trick::Clock::clock_init() */
        virtual int clock_init();

        /** @copybrief Trick::Clock::wall_clock_time() */
        virtual int64_t wall_clock_time();

        /** @copybrief Trick::Clock::clock_stop() */
        virtual int clock_stop();
};

}  // namespace Trick

#endif /* __EXTERNAL_CLOCK_SOURCE_HH__ */
