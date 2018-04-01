#ifndef EXE_SIL_MASTER_MODIFIED_DATA_NSPO_H_
#define EXE_SIL_MASTER_MODIFIED_DATA_NSPO_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_nspo() {
    Trick::DRAscii *drg = new Trick::DRAscii("nspo");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.001);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.time->gpstime.SOW");
    drg->add_variable("rkt.dynamics._SBEE[0]");
    drg->add_variable("rkt.dynamics._SBEE[1]");
    drg->add_variable("rkt.dynamics._SBEE[2]");
    drg->add_variable("rkt.dynamics._VBEE[0]");
    drg->add_variable("rkt.dynamics._VBEE[1]");
    drg->add_variable("rkt.dynamics._VBEE[2]");
    drg->add_variable("rkt.dynamics._ABEE[0]");
    drg->add_variable("rkt.dynamics._ABEE[1]");
    drg->add_variable("rkt.dynamics._ABEE[2]");
    drg->add_variable("rkt.dynamics._JBEE[0]");
    drg->add_variable("rkt.dynamics._JBEE[1]");
    drg->add_variable("rkt.dynamics._JBEE[2]");
    drg->add_variable("rkt.dynamics.psibd");
    drg->add_variable("rkt.dynamics.thtbd");
    drg->add_variable("rkt.dynamics.phibd");
    drg->add_variable("rkt.dynamics._WBEB[0]");
    drg->add_variable("rkt.dynamics._WBEB[1]");
    drg->add_variable("rkt.dynamics._WBEB[2]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_SIL_MASTER_MODIFIED_DATA_NSPO_H_
