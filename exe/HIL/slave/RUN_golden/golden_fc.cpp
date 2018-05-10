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
    // realtime();
    slave_init_time(&fc);
    /* INS */
    slave_init_ins_variable(&fc);
    /* GPS */
    slave_init_gps_fc_variable(&fc);

    slave_init_stage2_control(&fc);
    /* events */
    mission_event_trigger_configuration(&fc);

    return 0;
}
