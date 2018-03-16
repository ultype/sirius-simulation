#include "trick/external_clock_source.hh"
#include "trick/message_proto.h"
#include "trick/message_type.h"

/**
@details
-# Calls the base Clock constructor
*/
Trick::ExternalSourceClock::ExternalSourceClock() { }

/**
@details
-# This function is empty
*/
Trick::ExternalSourceClock::~ExternalSourceClock() { }

/**
@details
-# Set the global "the_clock" pointer to this instance
*/
int Trick::ExternalSourceClock::clock_init() {
    set_global_clock();
    return 0;
}

/**
@details
-# Call the system clock_gettime to get the current real time.
-# Return the current real time as a count of microseconds
*/
int64_t Trick::ExternalSourceClock::wall_clock_time() {
}

