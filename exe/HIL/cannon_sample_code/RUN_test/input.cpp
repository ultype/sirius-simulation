#include <cstdlib>
#include <exception>
#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

#include "../../../public/Modified_data/realtime.h"


extern "C" void external_clock_switch(void) {
    real_time_change_clock(&dyn.ext_clk);
    fprintf(stderr, "%s\n", real_time_clock_get_name());
}

extern "C" int run_me() {
    external_clock_switch();
    realtime();
    exec_set_terminate_time(40.0);

    return 0;
}
