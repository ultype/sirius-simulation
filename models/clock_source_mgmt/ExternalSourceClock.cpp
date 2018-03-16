#include "ExternalSourceClock.hh"
#include "trick/message_proto.h"
#include "trick/message_type.h"

/**
@details
-# Calls the base Clock constructor
*/
ExternalSourceClock::ExternalSourceClock() : Clock(1000000, "ExternalSourceClock")
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
    set_global_clock();
    return 0;
}

/**
@details
-# Call the system clock_gettime to get the current real time.
-# Return the current real time as a count of microseconds
*/
long long ExternalSourceClock::wall_clock_time() {
}

/**
@details
-# This function is empty
*/
int ExternalSourceClock::clock_stop() {
    return 0;
}
