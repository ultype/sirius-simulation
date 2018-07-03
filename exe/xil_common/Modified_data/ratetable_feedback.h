#ifndef EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_ratetable_feedback(double period) {
    Trick::DRAscii *drg = new Trick::DRAscii("ratetable_feedback");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(period);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input_deg[0]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input_deg[1]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_input_deg[2]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output_deg[0]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output_deg[1]");
    drg->add_variable("rkt.rt_dev_info.mot_data.hwil_output_deg[2]");

    drg->add_variable("rkt.adis16488_dev.log_data.accel_log[0]");
    drg->add_variable("rkt.adis16488_dev.log_data.accel_log[1]");
    drg->add_variable("rkt.adis16488_dev.log_data.accel_log[2]");
    drg->add_variable("rkt.adis16488_dev.log_data.angular_vel_log[0]");
    drg->add_variable("rkt.adis16488_dev.log_data.angular_vel_log[1]");
    drg->add_variable("rkt.adis16488_dev.log_data.angular_vel_log[2]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_RATETABLE_FEEDBACK_H_
