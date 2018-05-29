#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

#include "../../../xil_common/Modified_data/golden.h"
#include "../../../xil_common/Modified_data/nspo.h"
#include "../../../xil_common/Modified_data/gps.h"
#include "../../../xil_common/include/realtime.h"
#include "../../../xil_common/include/mission_event_handler.h"
#include "../../../xil_common/include/sirius_utility.h"

extern "C" int run_me() {
    record_nspo();
    record_gps();
    record_golden();
    //  external_clock_switch(&rkt.ext_clk);
    realtime();
    master_startup(&rkt);
    fprintf(stderr, "time_tic_value = %d tics per seconds\n", exec_get_time_tic_value());
    fprintf(stderr, "software_frame = %lf second per frame.\n", exec_get_software_frame());
    fprintf(stderr, "software_frame_tics = %lld tics per_frame\n", exec_get_software_frame_tics());
    master_model_configuration(&rkt);
    master_init_time(&rkt);
    master_init_environment(&rkt);
    master_init_slv(&rkt);
    master_init_aerodynamics(&rkt);
    master_init_propulsion(&rkt);
    master_init_sensors(&rkt);
    master_init_tvc(&rkt);
    mission_event_handler_configuration(&rkt);
    return 0;
}
