#include "utility_header.hh"

#include "Tvc.hh"
#include "sim_services/include/simtime.h"

void TVC::default_data(){
}

void TVC::initialize(Environment *env, Kinematics *kins, Control *con, Propulsion *plp){
    environment = env;
    kinematics = kins;
    control = con;
    propulsion = plp;
}

///////////////////////////////////////////////////////////////////////////////
// Actuator module
// Member function of class 'Hyper'
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting
//         =3 same as 2 but with on-line TVC gain
//
// This subroutine performs the following functions:
// (1) Converts from control commands to nozzle deflections
// (2) Calls nozzle dynamic subroutine
//
// 030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void TVC::actuate(double int_step){
    // local variables
    double eta(0), zet(0);
    double etac(0), zetc(0);

    // local module-variables
    Matrix FPB(3, 1);
    Matrix FMPB(3, 1);

    // input from other modules
    double pdynmc = environment->pdynmc;
    double xcg = propulsion->xcg;
    double thrust = propulsion->thrust;
    double delecx = control->delecx;
    double delrcx = control->delrcx;
    double alphax = kinematics->alphax;
    /*********************************
    Matrix UTBC=hyper[430].vec();
    double thtbdcx=hyper[339].real();
    double psibdcx=hyper[340].real();
    double thtvdx=hyper[229].real();
    int mguide=hyper[400].integer();
    ********************************/
    //-------------------------------------------------------------------------
    // return if no tvc
    if (mtvc == 0)
        return;

    // variable nozzle reduction gain (low q, high gain)
    if (mtvc == 3) {
        if (pdynmc > 1e5)
            gtvc = 0;
        else
            gtvc = (-5.e-6 * pdynmc + 0.5) * (factgtvc + 1);
    }
    // reduction of nozzle deflection command
    // if(mguide==5)
    //{
    // etac=0.01*(UTBC[2]-thtbdcx*RAD-0.5*PI);
    // zetc=0.0;//atan2(UTBC[1],sqrt(UTBC[2]*UTBC[2]+UTBC[0]*UTBC[0]));

    //}else{
    etac = gtvc * delecx * RAD;
    zetc = gtvc * delrcx * RAD;
    //}
    // no nozzle dynamics
    if (mtvc == 1) {
        eta = alphax * RAD;
        zet = zetc;
    }
    // calling second order nozzle dynamics
    if (mtvc >= 2)
        tvc_scnd(eta, zet, etac, zetc, int_step);

    // thrust forces in body axes
    double seta = sin(eta);
    double ceta = cos(eta);
    double czet = cos(zet);
    double szet = sin(zet);
    FPB[0] = ceta * czet * thrust;
    FPB[1] = ceta * szet * thrust;
    FPB[2] = -seta * thrust;

    // thrust moments in body axes
    double arm = parm - xcg;
    FMPB[0] = 0;
    FMPB[1] = arm * FPB[2];
    FMPB[2] = -arm * FPB[1];

    // output
    etax = eta * DEG;
    zetx = zet * DEG;

    // diagnostic
    etacx = etac * DEG;
    zetcx = zetc * DEG;

    //-------------------------------------------------------------------------
    // loading module-variables
    // output to other modules
    FPB.fill(fpb);
    FMPB.fill(fmpb);
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

void TVC::tvc_scnd(double &eta, double &zet, double etac, double zetc, double int_step){
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
    double etasd_new = detas;
    etas = integrate(etasd_new, etasd, etas, int_step);

    etasd = etasd_new;
    double eetas = etac - etas;
    double detasd_new = wntvc * wntvc * eetas - 2. * zettvc * wntvc * etasd;
    detas = integrate(detasd_new, detasd, detas, int_step);
    detasd = detasd_new;
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
    double zetad_new = dzeta;
    zeta = integrate(zetad_new, zetad, zeta, int_step);
    zetad = zetad_new;
    double ezeta = zetc - zeta;
    double dzetad_new = wntvc * wntvc * ezeta - 2. * zettvc * wntvc * zetad;
    dzeta = integrate(dzetad_new, dzetad, dzeta, int_step);
    dzetad = dzetad_new;
    // setting nozzle rate derivative to zero if rate is limited
    if (iflag && dzeta * dzetad > 0.)
        dzetad = 0.;
    zet = zeta;
}

