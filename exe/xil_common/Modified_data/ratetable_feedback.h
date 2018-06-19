#ifndef EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_ratetable_feedback() {
    Trick::DRAscii *drg = new Trick::DRAscii("ratetable_feedback");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.05);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input[0]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input[1]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input[2]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output[0]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output[1]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output[2]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_
