#include <cstdlib>
#include <exception>
#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

#include "../../../xil_common/include/realtime.h"

extern "C" int run_me() {
    //  external_clock_switch(&dyn.ext_clk);
    realtime();
    exec_set_terminate_time(200.0);
    return 0;
}
