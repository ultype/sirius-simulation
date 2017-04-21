#include "Rcs.hh"

#include <iostream>

RCS::RCS(Propulsion &plp)
    :   propulsion(&plp),
        roll_schi(0, 0),
        pitch_schi(0, 0),
        yaw_schi(0, 0),
        VECTOR_INIT(FMRCS, 3),
        VECTOR_INIT(FARCS, 3)
{
    this->default_data();
}

RCS::RCS(const RCS& other)
    :   propulsion(other.propulsion),
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

    this->roll_mom_max  = other.roll_mom_max;
    this->pitch_mom_max = other.pitch_mom_max;
    this->yaw_mom_max   = other.yaw_mom_max;
    this->rcs_thrust    = other.rcs_thrust;
    this->rocket_r      = other.rocket_r;
    this->rcs_pos       = other.rcs_pos;

    this->FMRCS = other.FMRCS;
    this->FARCS = other.FARCS;

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

    this->propulsion = other.propulsion;

    this->roll_schi = other.roll_schi;
    this->pitch_schi = other.pitch_schi;
    this->yaw_schi = other.yaw_schi;

    this->roll_mom_max  = other.roll_mom_max;
    this->pitch_mom_max = other.pitch_mom_max;
    this->yaw_mom_max   = other.yaw_mom_max;
    this->rcs_thrust    = other.rcs_thrust;
    this->rocket_r      = other.rocket_r;
    this->rcs_pos       = other.rcs_pos;

    this->FMRCS = other.FMRCS;
    this->FARCS = other.FARCS;

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
    o_roll = roll_schi.trigger(grab_e_roll());
    if (o_roll != o_roll_save) roll_count++;

    int o_pitch_save = o_pitch;
    o_pitch = pitch_schi.trigger(grab_e_pitch());
    if (o_pitch != o_pitch_save) pitch_count++;

    int o_yaw_save = o_yaw;
    o_yaw = yaw_schi.trigger(grab_e_yaw());
    if (o_yaw != o_yaw_save) yaw_count++;

    FMRCS(0) = o_roll  * rcs_thrust * rocket_r;
    FMRCS(1) = o_pitch * rcs_thrust * (propulsion->get_xcg() - 1);
    FMRCS(2) = o_yaw   * rcs_thrust * (propulsion->get_xcg() - 1);
}

void RCS::set_roll_mom_max(double in) { roll_mom_max = in; }
void RCS::set_pitch_mom_max(double in) { pitch_mom_max = in; }
void RCS::set_yaw_mom_max(double in) { yaw_mom_max = in; }
void RCS::set_rcs_thrust(double in) { rcs_thrust = in; }
void RCS::set_rocket_r(double in) { rocket_r = in; }
void RCS::set_rcs_pos(double in) { rcs_pos = in; }

arma::vec3 RCS::get_FMRCS(){ return FMRCS; }
arma::vec3 RCS::get_FARCS(){ return FARCS; }

