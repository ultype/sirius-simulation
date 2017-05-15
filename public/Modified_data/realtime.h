#include "trick/realtimesync_proto.h"
#include "trick/framelog_proto.h"
#include "trick/exec_proto.h"
extern "C" void realtime() {
    real_time_enable() ;
    exec_set_software_frame(0.05);
    trick_real_time.itimer.enable();
    exec_set_thread_priority(0, 1);
    exec_set_thread_cpu_affinity(0,1);
    frame_log_on();
}
