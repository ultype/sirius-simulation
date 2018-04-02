#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"
#include "../../../xil_common/include/realtime.h"
#include "../../../xil_common/Modified_data/gps_fc.h"
#include  "../../../models/gnc/include/DM_FSW_Interface.hh"
#include "../../../xil_common/include/mission_event_trigger.h"


extern "C" int run_me() {
    record_gps_slave();
    //  realtime();

    unsigned int Year = 2017;
    unsigned int DOY = 81;
    unsigned int Hour = 2;
    unsigned int Min = 0;
    unsigned int Sec = 0;
    fc.time->load_start_time(Year, DOY, Hour, Min, Sec);

    /* INS */
    init_ins_variable();
    /* GPS */
    init_gps_fc_variable();

    init_stage2_control();

    /* events */
    jit_add_read(0.001, "event_liftoff");
    jit_add_read(0.001, "event_control_rcs_on");
    jit_add_read(12.001, "event_control_on");
    //  jit_add_read(15.001, "event_s2_control_on");
    jit_add_read(82.001, "event_aoac_on");
    jit_add_read(100.001, "event_s3_control_on");
    jit_add_read(107.001, "event_fairing_jettison");
    jit_add_read(200.001, "event_control_off");

    exec_set_terminate_time(200.0);

    return 0;
}
