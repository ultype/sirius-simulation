#ifndef EXE_XIL_COMMON_MODIFIED_DATA_REPORT_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_REPORT_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_report() {
    Trick::DRAscii *drg = new Trick::DRAscii("report");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.005);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.env.pdynmc");
    drg->add_variable("rkt.dynamics._SBII[0]");
    drg->add_variable("rkt.dynamics._SBII[1]");
    drg->add_variable("rkt.dynamics._SBII[2]");
    drg->add_variable("rkt.dynamics._VBII[0]");
    drg->add_variable("rkt.dynamics._VBII[1]");
    drg->add_variable("rkt.dynamics._VBII[2]");
    drg->add_variable("rkt.dynamics.lonx");
    drg->add_variable("rkt.dynamics.latx");
    drg->add_variable("rkt.dynamics.psibdx");
    drg->add_variable("rkt.dynamics.thtbdx");
    drg->add_variable("rkt.dynamics.phibdx");
    drg->add_variable("rkt.dynamics._grndtrck");
    drg->add_variable("rkt.dynamics.alt");
    drg->add_variable("rkt.accelerometer->_FSPCB[0]");
    drg->add_variable("rkt.accelerometer->_FSPCB[1]");
    drg->add_variable("rkt.accelerometer->_FSPCB[2]");
    drg->add_variable("rkt.dynamics.alphax");
    drg->add_variable("rkt.dynamics.control_loss");
    drg->add_variable("rkt.dynamics._aero_loss");
    drg->add_variable("rkt.dynamics.gravity_loss");
    drg->add_variable("rkt.dynamics._dvbi");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_REPORT_H_
