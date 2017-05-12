#ifndef PUBLIC_MODIFIED_DATA_GOLDEN_H_
#define PUBLIC_MODIFIED_DATA_GOLDEN_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_golden() {
    Trick::DRAscii *drg = new Trick::DRAscii("rocket_csv");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.1);
    drg->set_single_prec_only(false);
    drg->add_variable("rkt.newton.lonx");
    drg->add_variable("rkt.newton.latx");
    drg->add_variable("rkt.newton.alt");
    drg->add_variable("rkt.newton._psivdx");
    drg->add_variable("rkt.newton._thtvdx");
    drg->add_variable("rkt.newton._dvbi");
    drg->add_variable("rkt.newton._SBII[0]");
    drg->add_variable("rkt.newton._SBII[1]");
    drg->add_variable("rkt.newton._SBII[2]");
    drg->add_variable("rkt.newton._VBII[0]");
    drg->add_variable("rkt.newton._VBII[1]");
    drg->add_variable("rkt.newton._VBII[2]");
    drg->add_variable("rkt.kinematics.alphax");
    drg->add_variable("rkt.kinematics.betax");
    drg->add_variable("rkt.env.atmosphere[0].density");
    drg->add_variable("rkt.env.vmach");
    drg->add_variable("rkt.env.pdynmc");
    drg->add_variable("rkt.propulsion.vmass");
    drg->add_variable("rkt.propulsion.fmassr");
    drg->add_variable("rkt.propulsion.thrust");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // PUBLIC_MODIFIED_DATA_GOLDEN_H_
