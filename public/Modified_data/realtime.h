#ifndef PUBLIC_MODIFIED_DATA_REALTIME_H_
#define PUBLIC_MODIFIED_DATA_REALTIME_H_

#include "trick/realtimesync_proto.h"
#include "trick/framelog_proto.h"

extern "C" void realtime() {
    real_time_enable();
    exec_set_software_frame(0.05);
    trick_real_time.itimer.enable();

    frame_log_on();
}

#endif  // PUBLIC_MODIFIED_DATA_REALTIME_H_
