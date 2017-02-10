#include "aux/utility_header.hh"
#include "rocket/Rcs.hh"

RCS::RCS(INS &i, Guidance &guia, Propulsion &plp)
    :   ins(&i), guidance(&guia), propulsion(&plp),
        VECTOR_INIT(FMRCS, 3),
        VECTOR_INIT(FARCS, 3)
{
    this->default_data();
}

RCS::RCS(const RCS& other)
    :   ins(other.ins), guidance(other.guidance), propulsion(other.propulsion),
        VECTOR_INIT(FMRCS, 3),
        VECTOR_INIT(FARCS, 3)
{
    this->default_data();

}

RCS& RCS::operator=(const RCS& other){
    if(&other == this)
        return *this;

    this->ins = other.ins;
    this->guidance = other.guidance;
    this->propulsion = other.propulsion;

    return *this;
}

void RCS::default_data(){
}

void RCS::initialize(){
}

void RCS::enable_rcs(){
    rcs_type = ON_OFF_RCS;
}

void RCS::disable_rcs(){
    rcs_type = NO_RCS;
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

    // moments generated about the three principle axes wrt the c.m
    //FMRCS[0]=o_roll*roll_mom_max;
    //FMRCS[1]=o_pitch*pitch_mom_max;
    //FMRCS[2]=o_yaw*yaw_mom_max;
    //FMRCS[0]=o_roll*rcs_thrust*rocket_r;
    //FMRCS[1]=o_pitch*rcs_thrust*(xcg-1);
    //FMRCS[2]=o_yaw*rcs_thrust*(xcg-1);

    //on-off side force thrusters (Schmitt trigger)

    //acceleration control
    //double ay=FSPCB[1];
    //e_right=acc_gain*(aycomx*AGRAV-ay);

    //int o_right_save=o_right;
    //o_right=rcs_schmitt(e_right,right_save,dead_zone,hysteresis);
    //right_save=e_right;
    //if(o_right!=o_right_save)right_count++;

    //double az=FSPCB[2];
    //e_down=acc_gain*(azcomx*AGRAV-az);
    //int o_down_save=o_down;
    //o_down=rcs_schmitt(e_down,down_save,dead_zone,hysteresis);
    //down_save=e_down;
    //if(o_down!=o_down_save)down_count++;

    //FARCS[0]=0;
    //FARCS[1]=o_right*side_force_max;
    //FARCS[2]=o_down*side_force_max;
}

///////////////////////////////////////////////////////////////////////////////
// Proportional thrusters with limiters
// Member function of class 'Hyper'
//
// parameter input
//    input = commanded input - Nm
//    limiter = max and min value of applied moment - Nm
// return output
//    output = moment about principle axis through c.m. - Nm
//
// 040309 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
/**************************************************
double Hyper::rcs_prop(double input,double limiter)
{
    //local variable
    double output(0);
    output=input;
    if(fabs(input)>limiter)
        output=limiter*sign(input);

    return output;
}
****************************************************/

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
    int o_roll_save = o_roll;
    o_roll = rcs_schmitt(e_roll, roll_save, dead_zone, hysteresis);
    roll_save = e_roll;
    if (o_roll != o_roll_save)
        roll_count++;

    int o_pitch_save = o_pitch;
    o_pitch = rcs_schmitt(e_pitch, pitch_save, dead_zone, hysteresis);
    pitch_save = e_pitch;
    if (o_pitch != o_pitch_save)
        pitch_count++;

    int o_yaw_save = o_yaw;
    o_yaw = rcs_schmitt(e_yaw, yaw_save, dead_zone, hysteresis);
    yaw_save = e_yaw;
    if (o_yaw != o_yaw_save)
        yaw_count++;

    FMRCS(0) = o_roll  * rcs_thrust * rocket_r;
    FMRCS(1) = o_pitch * rcs_thrust * (propulsion->get_xcg() - 1);
    FMRCS(2) = o_yaw   * rcs_thrust * (propulsion->get_xcg() - 1);
}

int RCS::rcs_schmitt(double input_new, double input, double dead_zone, double hysteresis) {
    // local variable
    int output(0);

    // input signal 'trend' (=1 increasing, =-1 decreasing)
    // input signal 'side' (=-1 left, =1 right)
    int trend = sign(input_new - input);
    int side = sign(input);
    double trigger = (dead_zone * side + hysteresis * trend) / 2;

    if (input >= trigger && side == 1) {
        output =  1;
    } else if (input <= trigger && side == -1) {
        output = -1;
    } else
        output = 0;

    return output;
}

void RCS::set_rcs_tau(double in) { rcs_tau = in; }
void RCS::set_roll_mom_max(double in) { roll_mom_max = in; }
void RCS::set_pitch_mom_max(double in) { pitch_mom_max = in; }
void RCS::set_yaw_mom_max(double in) { yaw_mom_max = in; }
void RCS::set_thtbdcomx(double in) { thtbdcomx = in; }
void RCS::set_psibdcomx(double in) { psibdcomx = in; }
void RCS::set_dead_zone(double in) { dead_zone = in; }
void RCS::set_hysteresis(double in) { hysteresis = in; }
void RCS::set_rcs_thrust(double in) { rcs_thrust = in; }
void RCS::set_rocket_r(double in) { rocket_r = in; }
void RCS::set_rcs_pos(double in) { rcs_pos = in; }

enum RCS::RCS_MODE RCS::get_rcs_mode() { return rcs_mode; }

Matrix RCS::get_FMRCS() {
    Matrix FMRCS(_FMRCS);
    return FMRCS;
}
Matrix RCS::get_FARCS() {
    Matrix FARCS(_FARCS);
    return FARCS;
}
