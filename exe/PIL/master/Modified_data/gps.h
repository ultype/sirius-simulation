#ifndef EXE_PIL_MASTER_MODIFIED_DATA_GPS_H_
#define EXE_PIL_MASTER_MODIFIED_DATA_GPS_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_gps() {
    Trick::DRAscii *drg = new Trick::DRAscii("gps");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.05);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.time->gpstime.SOW");
    drg->add_variable("rkt.gps_con.gdop");
    drg->add_variable("rkt.gps_con.nsat");
    drg->add_variable("rkt.newton._SBII[0]");
    drg->add_variable("rkt.newton._SBII[1]");
    drg->add_variable("rkt.newton._SBII[2]");
    drg->add_variable("rkt.newton._SBEE[0]");
    drg->add_variable("rkt.newton._SBEE[1]");
    drg->add_variable("rkt.newton._SBEE[2]");
    drg->add_variable("rkt.newton._VBEE[0]");
    drg->add_variable("rkt.newton._VBEE[1]");
    drg->add_variable("rkt.newton._VBEE[2]");
    drg->add_variable("rkt.gps_con.chan[0].prn");
    drg->add_variable("rkt.gps_con.chan[0].rho0.range");
    drg->add_variable("rkt.gps_con.chan[0].rho0.rate");
    drg->add_variable("rkt.gps_con.chan[1].prn");
    drg->add_variable("rkt.gps_con.chan[1].rho0.range");
    drg->add_variable("rkt.gps_con.chan[1].rho0.rate");
    drg->add_variable("rkt.gps_con.chan[2].prn");
    drg->add_variable("rkt.gps_con.chan[2].rho0.range");
    drg->add_variable("rkt.gps_con.chan[2].rho0.rate");
    drg->add_variable("rkt.gps_con.chan[3].prn");
    drg->add_variable("rkt.gps_con.chan[3].rho0.range");
    drg->add_variable("rkt.gps_con.chan[3].rho0.rate");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_PIL_MASTER_MODIFIED_DATA_GPS_H_
