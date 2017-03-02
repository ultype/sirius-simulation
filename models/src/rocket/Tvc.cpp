#include "aux/utility_header.hh"
#include "math/utility.hh"
#include "math/integrate.hh"

#include "rocket/Tvc.hh"
#include "sim_services/include/simtime.h"

TVC::TVC(Environment &env, Kinematics &kins, Control &con, Propulsion &plp)
    :   environment(&env), kinematics(&kins), control(&con), propulsion(&plp),
        VECTOR_INIT(FPB, 3),
        VECTOR_INIT(FMPB, 3)
{
    this->default_data();
}

TVC::TVC(const TVC& other)
    :   environment(other.environment), kinematics(other.kinematics), control(other.control), propulsion(other.propulsion),
        VECTOR_INIT(FPB, 3),
        VECTOR_INIT(FMPB, 3)
{
    this->default_data();


    this->mtvc = other.mtvc;

    /* Constants */
    this->gtvc = other.gtvc;

    this->tvclimx = other.tvclimx;
    this->dtvclimx = other.dtvclimx;
    this->wntvc = other.wntvc;
    this->zettvc = other.zettvc;
    this->factgtvc = other.factgtvc;

    /* Propagative Stats */
    this->etas = other.etas;
    this->etasd = other.etasd;

    this->zeta = other.zeta;
    this->zetad = other.zetad;

    this->detas = other.detas;
    this->detasd = other.detasd;

    this->dzeta = other.dzeta;
    this->dzetad = other.dzetad;

    /* Generating Outputs */
    this->parm = other.parm;

    this->FPB = other.FPB;

    this->FMPB = other.FMPB;

    this->etax = other.etax;
    this->zetx = other.zetx;

    /* Non-propagating Diagnostic Variables */
    /* These can be deleted, but keep to remain trackable in trick simulator */
    this->etacx = other.etacx;
    this->zetcx = other.zetcx;
}

TVC& TVC::operator=(const TVC& other){
    if(&other == this)
        return *this;

    this->environment = other.environment;
    this->kinematics = other.kinematics;
    this->control = other.control;
    this->propulsion = other.propulsion;

    this->mtvc = other.mtvc;

    /* Constants */
    this->gtvc = other.gtvc;

    this->tvclimx = other.tvclimx;
    this->dtvclimx = other.dtvclimx;
    this->wntvc = other.wntvc;
    this->zettvc = other.zettvc;
    this->factgtvc = other.factgtvc;

    /* Propagative Stats */
    this->etas = other.etas;
    this->etasd = other.etasd;

    this->zeta = other.zeta;
    this->zetad = other.zetad;

    this->detas = other.detas;
    this->detasd = other.detasd;

    this->dzeta = other.dzeta;
    this->dzetad = other.dzetad;

    /* Generating Outputs */
    this->parm = other.parm;

    this->FPB = other.FPB;

    this->FMPB = other.FMPB;

    this->etax = other.etax;
    this->zetx = other.zetx;

    /* Non-propagating Diagnostic Variables */
    /* These can be deleted, but keep to remain trackable in trick simulator */
    this->etacx = other.etacx;
    this->zetcx = other.zetcx;

    return *this;
}

void TVC::default_data(){
}

void TVC::initialize(){
}

///////////////////////////////////////////////////////////////////////////////
// Actuator module
// Member function of class 'Hyper'
//
// This subroutine performs the following functions:
// (1) Converts from control commands to nozzle deflections
// (2) Calls nozzle dynamic subroutine
//
// 030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

arma::vec3 TVC::calculate_FPB(double eta, double zet, double thrust){
    arma::vec3 __FPB;

    double seta = sin(eta);
    double ceta = cos(eta);
    double czet = cos(zet);
    double szet = sin(zet);
    __FPB[0] = ceta * czet * thrust;
    __FPB[1] = ceta * szet * thrust;
    __FPB[2] = -seta * thrust;

    return __FPB;
}

arma::vec3 TVC::calculate_FMPB(double xcg){
    arma::vec3 __FMPB;

    double arm = parm - xcg;
    __FMPB[0] = 0;
    __FMPB[1] = arm * FPB[2];
    __FMPB[2] = -arm * FPB[1];

    return __FMPB;
}

void TVC::actuate(double int_step){
    // local variables
    double eta(0), zet(0);
    double etac(0), zetc(0);

    // input from other modules
    double pdynmc = environment->get_pdynmc();
    double xcg    = propulsion->get_xcg();
    double thrust = propulsion->get_thrust();
    double delecx = control->get_delecx();
    double delrcx = control->get_delrcx();
    double alphax = kinematics->get_alphax();

    switch(mtvc){
        case NO_TVC:
            // return if no tvc
            return;
            break;
        case NO_DYNAMIC_TVC:
            eta = alphax * RAD;
            zet = zetc;
            break;
        case ONLINE_SECOND_ORDER_TVC:
            // variable nozzle reduction gain (low q, high gain)
            if (pdynmc > 1e5)
                gtvc = 0;
            else
                gtvc = (-5.e-6 * pdynmc + 0.5) * (factgtvc + 1);
        case SECON_ORDER_TVC:
            etac = gtvc * delecx * RAD;
            zetc = gtvc * delrcx * RAD;
            // calling second order nozzle dynamics
            std::tie(eta, zet) = tvc_scnd(etac, zetc, int_step);
            break;
        default:
            break;
    }

    // thrust forces in body axes
    this->FPB = calculate_FPB(eta, zet, thrust);

    // thrust moments in body axes
    this->FMPB = calculate_FMPB(xcg);

    // output
    etax = eta * DEG;
    zetx = zet * DEG;

    // diagnostic
    etacx = etac * DEG;
    zetcx = zetc * DEG;
}

///////////////////////////////////////////////////////////////////////////////
// Second order TVC
// Member function of class 'Hyper'
// This subroutine performs the following functions:
// (1) Models second order lags of pitch and yaw deflections
// (2) Limits nozzle deflections
// (3) Limits nozzle deflection rates
//
// Return output
//          eta=Nozzle pitch deflection - rad
//          zet=Nozzle yaw deflection - rad
// Argument Input:
//          etac=Nozzle pitch command - rad
//          zetc=Nozzle yaw command - rad
//
// 030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

std::tuple<double, double> TVC::tvc_scnd(double etac, double zetc, double int_step){
    double eta, zet;

    // pitch nozzle dynamics
    // limiting position and the nozzle rate derivative
    if (fabs(etas) > tvclimx * RAD) {
        etas = tvclimx * RAD * sign(etas);
        if (etas * detas > 0.)
            detas = 0.;
    }
    // limiting nozzle rate
    int iflag = 0;
    if (fabs(detas) > dtvclimx * RAD) {
        iflag = 1;
        detas = dtvclimx * RAD * sign(detas);
    }
    // state integration
    INTEGRATE(etas, detas);

    double eetas = etac - etas;

    INTEGRATE(detas, wntvc * wntvc * eetas - 2. * zettvc * wntvc * etasd);

    // setting nozzle rate derivative to zero if rate is limited
    if (iflag && detas * detasd > 0.)
        detasd = 0.;
    eta = etas;

    // yaw nozzle dynamics
    // limiting position and the nozzle rate derivative
    if (fabs(zeta) > tvclimx * RAD) {
        zeta = tvclimx * RAD * sign(zeta);
        if (zeta * dzeta > 0.)
            dzeta = 0.;
    }
    // limiting nozzle rate
    iflag = 0;
    if (fabs(dzeta) > dtvclimx * RAD) {
        iflag = 1;
        dzeta = dtvclimx * RAD * sign(dzeta);
    }
    // state integration
    INTEGRATE(zeta, dzeta);

    double ezeta = zetc - zeta;

    INTEGRATE(dzeta, wntvc * wntvc * ezeta - 2. * zettvc * wntvc * zetad);

    // setting nozzle rate derivative to zero if rate is limited
    if (iflag && dzeta * dzetad > 0.)
        dzetad = 0.;
    zet = zeta;

    return std::make_tuple(eta, zet);
}

enum TVC::TVC_TYPE TVC::get_mtvc() { return mtvc; }
void TVC::set_mtvc(enum TVC_TYPE in) { mtvc = in; }

double TVC::get_gtvc() { return gtvc;  }
void TVC::set_gtvc(double in) { gtvc = in; }

void TVC::set_tvclimx(double in) { tvclimx = in; }
void TVC::set_dtvclimx(double in) { dtvclimx = in; }
void TVC::set_wntvc(double in) { wntvc = in; }
void TVC::set_zettvc(double in) { zettvc = in; }
void TVC::set_factgtvc(double in) { factgtvc = in; }

double TVC::get_parm() { return parm; }

arma::vec3 TVC::get_FPB() { return FPB; }
arma::vec3 TVC::get_FMPB() { return FMPB; }
