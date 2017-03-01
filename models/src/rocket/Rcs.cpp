#include "aux/utility_header.hh"
#include "rocket/Rcs.hh"

#include <iostream>

RCS::RCS(INS &i, Guidance &guia, Propulsion &plp)
    :   ins(&i), guidance(&guia), propulsion(&plp),
        roll_schi(0, 0),
        pitch_schi(0, 0),
        yaw_schi(0, 0),
        VECTOR_INIT(FMRCS, 3),
        VECTOR_INIT(FARCS, 3)
{
    this->default_data();
}

RCS::RCS(const RCS& other)
    :   ins(other.ins), guidance(other.guidance), propulsion(other.propulsion),
        roll_schi(other.roll_schi),
        pitch_schi(other.pitch_schi),
        yaw_schi(other.yaw_schi),
        VECTOR_INIT(FMRCS, 3),
        VECTOR_INIT(FARCS, 3)
{
    this->default_data();

    this->roll_schi = other.roll_schi;
    this->pitch_schi = other.pitch_schi;
    this->yaw_schi = other.yaw_schi;

    this->thtbdcomx = other.thtbdcomx;
    this->psibdcomx = other.psibdcomx;
    this->phibdcomx = other.phibdcomx;

    this->rcs_type = other.rcs_type;
    this->rcs_mode = other.rcs_mode;

    this->rcs_tau       = other.rcs_tau;
    this->roll_mom_max  = other.roll_mom_max;
    this->pitch_mom_max = other.pitch_mom_max;
    this->yaw_mom_max   = other.yaw_mom_max;
    this->rcs_thrust    = other.rcs_thrust;
    this->rocket_r      = other.rocket_r;
    this->rcs_pos       = other.rcs_pos;

    this->FMRCS = other.FMRCS;
    this->FARCS = other.FARCS;

    this->e_roll  = other.e_roll;
    this->e_pitch = other.e_pitch;
    this->e_yaw   = other.e_yaw;

    this->o_roll  = other.o_roll;
    this->o_pitch = other.o_pitch;
    this->o_yaw   = other.o_yaw;

    this->roll_count  = other.roll_count;
    this->pitch_count = other.pitch_count;
    this->yaw_count   = other.yaw_count;
}

RCS& RCS::operator=(const RCS& other){
    if(&other == this)
        return *this;

    this->ins = other.ins;
    this->guidance = other.guidance;
    this->propulsion = other.propulsion;

    this->roll_schi = other.roll_schi;
    this->pitch_schi = other.pitch_schi;
    this->yaw_schi = other.yaw_schi;

    this->thtbdcomx = other.thtbdcomx;
    this->psibdcomx = other.psibdcomx;
    this->phibdcomx = other.phibdcomx;

    this->rcs_type = other.rcs_type;
    this->rcs_mode = other.rcs_mode;

    this->rcs_tau       = other.rcs_tau;
    this->roll_mom_max  = other.roll_mom_max;
    this->pitch_mom_max = other.pitch_mom_max;
    this->yaw_mom_max   = other.yaw_mom_max;
    this->rcs_thrust    = other.rcs_thrust;
    this->rocket_r      = other.rocket_r;
    this->rcs_pos       = other.rcs_pos;

    this->FMRCS = other.FMRCS;
    this->FARCS = other.FARCS;

    this->e_roll  = other.e_roll;
    this->e_pitch = other.e_pitch;
    this->e_yaw   = other.e_yaw;

    this->o_roll  = other.o_roll;
    this->o_pitch = other.o_pitch;
    this->o_yaw   = other.o_yaw;

    this->roll_count  = other.roll_count;
    this->pitch_count = other.pitch_count;
    this->yaw_count   = other.yaw_count;

    return *this;
}

void RCS::default_data(){
}

void RCS::initialize(){
}

void RCS::setup_rcs_schmitt_trigger(double dead_zone, double hysteresis){
    this->roll_schi = Schmitt_Trigger(dead_zone, hysteresis);
    this->pitch_schi = Schmitt_Trigger(dead_zone, hysteresis);
    this->yaw_schi = Schmitt_Trigger(dead_zone, hysteresis);

    this->roll_schi.clear();
    this->pitch_schi.clear();
    this->yaw_schi.clear();
}

void RCS::enable_rcs(){
    rcs_type = ON_OFF_RCS;
}

void RCS::disable_rcs(){
    rcs_type = NO_RCS;
}

bool RCS::isEnabled(){
    return rcs_type > 0;
}

void RCS::set_mode(enum RCS::RCS_MODE in){
    rcs_mode = in;
}

///////////////////////////////////////////////////////////////////////////////
// RCS thruster module
// Member function of class 'Hyper'
// Calls thrusters dynamic subroutine
//
// rcs_mode = 0 no control
//          = 1 all geodetic Euler angle control
//          = 2 thrust vector direction and roll angle control
//          = 3 incidence and roll angle controml
//          = 4 geodetic pitch angle control
//
//
// 040302 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void RCS::actuate(){
    arma::vec3 UTBC = arma::vec3(guidance->get_UTBC().get_pbody());
    double alphacomx = guidance->get_alphacomx();
    double betacomx  = guidance->get_betacomx();

    double qqcx = ins->get_gyro().get_qqcx();
    double ppcx = ins->get_gyro().get_ppcx();
    double rrcx = ins->get_gyro().get_rrcx();

    double alphacx = ins->get_alphacx();
    double betacx = ins->get_betacx();

    double thtbdcx = ins->get_thtbdcx();
    double psibdcx = ins->get_psibdcx();
    double phibdcx = ins->get_phibdcx();

    if(this->rcs_type == NO_RCS) return;

    // on-off moment thrusters (Schmitt trigger)
    // roll angle control (always)
    e_roll = phibdcomx - (rcs_tau * ppcx + phibdcx);

    // on/off output of Schmitt trigger
    switch(rcs_mode){
        case NO_CONTROL:
            break;

        case ALL_GEODETIC_EULUR_ANGLE_CONTROL:
            e_pitch = thtbdcomx - (rcs_tau * qqcx + thtbdcx);
            e_yaw   = psibdcomx - (rcs_tau * rrcx + psibdcx);
            break;

        case THRUST_VECTOR_DIRECTION_AND_ROLL_ANGLE_CONTROL:
            e_pitch = -rcs_tau * qqcx - UTBC[2] * DEG;
            e_yaw   = -rcs_tau * rrcx + UTBC[1] * DEG;
            break;

        case INCIDENCE_AND_ROLL_ANGLE_CONTROL:
            e_pitch = alphacomx - (rcs_tau * qqcx + alphacx);
            e_yaw   = -betacomx - (rcs_tau * rrcx - betacx);
            break;

        case GEODETIC_YAW_ANGLE_CONTROL:
            // e_yaw=psibdcomx-(rcs_tau*rrcx+psibdcx);
            e_pitch = thtbdcomx - (rcs_tau * qqcx + thtbdcx);
            break;

    }

}

///////////////////////////////////////////////////////////////////////////////
// On-off thrusters modeled by the Schmitt trigger
// Ref: Bryson,A.E.,"Control of Spacecraft and Aircraft",
//      Princeton University Press, 1994
//
// Switching mechanism with dead-zone and hysteresis, includes also pure
// hysteresis (dead_zone=0) pure dead-zone (hysteresis=0), or simple on/off
// relay
// (dead_zone=0,hysteresis=0)
// Member function of class 'Hyper'
//
// parameter input
//    input_new = new input value, state variable
//    input = previous input value, saved state variable
//    dead_zone = dead zone of trigger,
//                if zero -> pure hysteresis - units of input
//    hysteresis = hysteresis,
//                 if zero -> pure dead zone - units of input return output
//    =+/-1: trigger on; =0 trigger off
//
// 040308 Created by Peter H Zipfel
//////////////////////////////////////////////////////////////////////////////

void RCS::rcs_schmitt_thrust() {
    /* ------------------------------------------------------------ /
    / moments generated about the three principle axes wrt the c.m  /
    / Equations:                                                    /
    / FMRCS[0] = o_roll*roll_mom_max;                               /
    / FMRCS[1] = o_pitch*pitch_mom_max;                             /
    / FMRCS[2] = o_yaw*yaw_mom_max;                                 /
    /                                                               /
    / FMRCS[0] = o_roll*rcs_thrust*rocket_r;                        /
    / FMRCS[1] = o_pitch*rcs_thrust*(xcg-1);                        /
    / FMRCS[2] = o_yaw*rcs_thrust*(xcg-1);                          /
    / ------------------------------------------------------------ */

    int o_roll_save = o_roll;
    o_roll = roll_schi.trigger(e_roll);
    if (o_roll != o_roll_save) roll_count++;

    int o_pitch_save = o_pitch;
    o_pitch = pitch_schi.trigger(e_pitch);
    if (o_pitch != o_pitch_save) pitch_count++;

    int o_yaw_save = o_yaw;
    o_yaw = yaw_schi.trigger(e_yaw);
    if (o_yaw != o_yaw_save) yaw_count++;

    FMRCS(0) = o_roll  * rcs_thrust * rocket_r;
    FMRCS(1) = o_pitch * rcs_thrust * (propulsion->get_xcg() - 1);
    FMRCS(2) = o_yaw   * rcs_thrust * (propulsion->get_xcg() - 1);
}

void RCS::set_rcs_tau(double in) { rcs_tau = in; }
void RCS::set_roll_mom_max(double in) { roll_mom_max = in; }
void RCS::set_pitch_mom_max(double in) { pitch_mom_max = in; }
void RCS::set_yaw_mom_max(double in) { yaw_mom_max = in; }
void RCS::set_thtbdcomx(double in) { thtbdcomx = in; }
void RCS::set_psibdcomx(double in) { psibdcomx = in; }
void RCS::set_rcs_thrust(double in) { rcs_thrust = in; }
void RCS::set_rocket_r(double in) { rocket_r = in; }
void RCS::set_rcs_pos(double in) { rcs_pos = in; }

enum RCS::RCS_MODE RCS::get_rcs_mode() { return rcs_mode; }

arma::vec3 RCS::get_FMRCS(){ return FMRCS; }
arma::vec3 RCS::get_FARCS(){ return FARCS; }
