#ifndef EXE_XIL_COMMON_MODIFIED_DATA_GOLDEN_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_GOLDEN_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_golden() {
    Trick::DRAscii *drg = new Trick::DRAscii("rocket_csv");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.1);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.dynamics.lonx");
    drg->add_variable("rkt.dynamics.latx");
    drg->add_variable("rkt.dynamics.alt");
    drg->add_variable("rkt.dynamics._dvbi");
    drg->add_variable("rkt.dynamics._SBII[0]");
    drg->add_variable("rkt.dynamics._SBII[1]");
    drg->add_variable("rkt.dynamics._SBII[2]");
    drg->add_variable("rkt.dynamics._VBII[0]");
    drg->add_variable("rkt.dynamics._VBII[1]");
    drg->add_variable("rkt.dynamics._VBII[2]");
    drg->add_variable("rkt.dynamics.thtbdx");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_GOLDEN_H_
