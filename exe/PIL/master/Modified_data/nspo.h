#ifndef EXE_PIL_MASTER_MODIFIED_DATA_NSPO_H_
#define EXE_PIL_MASTER_MODIFIED_DATA_NSPO_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_nspo() {
    Trick::DRAscii *drg = new Trick::DRAscii("nspo");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.001);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.newton._SBEE[0]");
    drg->add_variable("rkt.newton._SBEE[1]");
    drg->add_variable("rkt.newton._SBEE[2]");
    drg->add_variable("rkt.newton._VBEE[0]");
    drg->add_variable("rkt.newton._VBEE[1]");
    drg->add_variable("rkt.newton._VBEE[2]");
    drg->add_variable("rkt.newton._ABEE[0]");
    drg->add_variable("rkt.newton._ABEE[1]");
    drg->add_variable("rkt.newton._ABEE[2]");
    drg->add_variable("rkt.newton._JBEE[0]");
    drg->add_variable("rkt.newton._JBEE[1]");
    drg->add_variable("rkt.newton._JBEE[2]");
    drg->add_variable("rkt.kinematics.psibd");
    drg->add_variable("rkt.kinematics.thtbd");
    drg->add_variable("rkt.kinematics.phibd");
    drg->add_variable("rkt.euler._WBEB[0]");
    drg->add_variable("rkt.euler._WBEB[1]");
    drg->add_variable("rkt.euler._WBEB[2]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_PIL_MASTER_MODIFIED_DATA_NSPO_H_
