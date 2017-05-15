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
    drg->add_variable("fc.gps.state_pos");
    drg->add_variable("fc.gps.state_vel");
    drg->add_variable("fc.ins.ins_pos_err");
    drg->add_variable("fc.gps.state_pos");
    drg->add_variable("fc.ins.ins_vel_err");;
    drg->add_variable("fc.ins.ins_pose_err");
    drg->add_variable("fc.ins.ins_vele_err");
    drg->add_variable("fc.ins.ins_phi_err");
    drg->add_variable("fc.ins.ins_tht_err");
    drg->add_variable("fc.ins.thtbdcx");
    drg->add_variable("fc.ins.phibdcx");
    drg->add_variable("fc.ins.psibdcx");
    drg->add_variable("fc.ins.ins_psi_err");
    drg->add_variable("rkt.newton._SBII[0]");
    drg->add_variable("rkt.newton._SBII[1]");
    drg->add_variable("rkt.newton._SBII[2]");
    drg->add_variable("fc.ins._WBICI[0]");
    drg->add_variable("fc.ins._WBICI[1]");
    drg->add_variable("fc.ins._WBICI[2]");
    drg->add_variable("rkt.newton._SBEE[0]");
    drg->add_variable("rkt.newton._SBEE[1]");
    drg->add_variable("rkt.newton._SBEE[2]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_PIL_MASTER_MODIFIED_DATA_GPS_H_
