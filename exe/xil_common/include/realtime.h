#ifndef EXE_XIL_COMMON_INCLUDE_REALTIME_H_
#define EXE_XIL_COMMON_INCLUDE_REALTIME_H_

#include "trick/realtimesync_proto.h"
#include "trick/framelog_proto.h"
#include "trick/exec_proto.h"
extern "C" void realtime() {
    real_time_enable();
    exec_set_software_frame(0.005);
    //  trick_real_time.itimer.enable();
    exec_set_lock_memory(1);
    exec_set_thread_priority(0, 1);
    exec_set_thread_cpu_affinity(0, 1);
    frame_log_on();
    fprintf(stderr, "%s\n", real_time_clock_get_name());
    fprintf(stderr, "S_main_Linux_5.4_x86_64.exe Process ID : %d\n", getpid());
}

extern "C" void freeze_cmd(void) {
    exec_set_freeze_command(1);
    exec_set_enable_freeze(1);
    sim_control_panel_set_enabled(1);
}

extern "C" void external_clock_switch(Trick::Clock * in_clock) {
    real_time_change_clock(in_clock);
    fprintf(stderr, "%s\n", real_time_clock_get_name());
}

#endif  // EXE_XIL_COMMON_INCLUDE_REALTIME_H_
