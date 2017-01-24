#include "Actuator.hh"

void Actuator::default_data(){
}

void Actuator::initialize(Control *con){
    control = con;
}

///////////////////////////////////////////////////////////////////////////////
// Actuator module
// Member function of class 'Hyper'
// Converts from control deflections to fin deflections
// Calls actuator dynamic subroutine
// Limits fins excursions and converts back to control deflections, as
// applicable
//
// mact=|morder|mvehicle|
//
//    morder = 0 no dynamics, fins are position limited
//           = 2 second order dynamics, fins are position and rate limited
//    mvehicle = 1 not used
//             = 2 Rocket
//
// 030515 Created by Peter H Zipfel
// 050405 New formulation,  1-6 fin actuator model, PZi
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Actuator::actuate(double int_step){
    // local variable
    Matrix ACTCZ(6, 1);
    Matrix ACTZ(6, 1);
    double delcx1(0);
    double delcx2(0);
    double delcx3(0);
    double delcx4(0);

    // input from other modules
    double delacx = 0; //XXX: Ambiguous;
    double delecx = control->get_delecx();
    double delrcx = control->get_delrcx();
    //-------------------------------------------------------------------------
    // decoding actuator flag
    int morder = mact / 10;
    mvehicle = (mact % 10);

    // Rocket
    if (mvehicle == 2) {
        num_fins = 4;
        dlimx_min = -dlimx;
        // converting to four fin deflections
        delcx1 = -delacx + delecx - delrcx;
        delcx2 = -delacx + delecx + delrcx;
        delcx3 = +delacx + delecx - delrcx;
        delcx4 = +delacx + delecx + delrcx;
        ACTCZ.assign_loc(0, 0, delcx1);
        ACTCZ.assign_loc(1, 0, delcx2);
        ACTCZ.assign_loc(2, 0, delcx3);
        ACTCZ.assign_loc(3, 0, delcx4);

        if (morder == 0)
            // no dynamics
            ACTZ = actuator_0th(ACTCZ, dlimx, dlimx_min, num_fins);
        else if (morder == 2)
            // second order dynamics
            ACTZ = actuator_scnd(ACTCZ, dlimx, dlimx_min, num_fins, int_step);

        delx1 = ACTZ.get_loc(0, 0);
        delx2 = ACTZ.get_loc(1, 0);
        delx3 = ACTZ.get_loc(2, 0);
        delx4 = ACTZ.get_loc(3, 0);
        // converting to control deflections
        delax = (-delx1 - delx2 + delx3 + delx4) / 4;
        delex = (+delx1 + delx2 + delx3 + delx4) / 4;
        delrx = (-delx1 + delx2 - delx3 + delx4) / 4;
    }
    //-------------------------------------------------------------------------
}

///////////////////////////////////////////////////////////////////////////////
// No actuator dynamics
// Member function of class 'Hyper'
// Models 6 control surfaces with position limiters
//
// Return output
//      ACTZ(6x1) = 6 control surface deflections (some may be zero, unused)
// Parameter input
//    ACTCZ(6x1) = 6 control surfaces commanded deflections (some may be zero,
//                   unused)
//
// 050406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Actuator::actuator_0th(Matrix ACTCZ, double dlimx, double dlimx_min, int num_fins){
    Matrix ACTZ = ACTCZ;
    // limiting surface deflections
    for (int i = 0; i < num_fins; i++) {
        // limiting position and the fin rate derivative
        if (ACTZ[i] > dlimx) {
            ACTZ[i] = dlimx;
        }
        if (ACTZ[i] < dlimx_min) {
            ACTZ[i] = dlimx_min;
        }
    }
    return ACTZ;
}
///////////////////////////////////////////////////////////////////////////////
// Second order actuator
// Member function of class 'Hyper'
// Models second order lags of up to 6 control surfaces
// Limits fin positions
// Limits fin rates
//
// Return output
//    ACTZ(6x1) = 6 control surface deflections (some may be zero, unused)
// Parameter input
//    ACTCZ(6x1) = 6 control surfaces commanded deflections (some may be zero,
//                 unused)
//
// 050405 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Actuator::actuator_scnd(Matrix ACTCZ, double dlimx, double dlimx_min, int num_fins, double int_step) {
    // local variables
    Matrix DZD_NEW(6, 1);
    Matrix DDZD_NEW(6, 1);
    Matrix ACTZ(6, 1);
    Matrix DZD(6, 1);
    Matrix DZ(6, 1);
    Matrix DDZD(6, 1);
    Matrix DDZ(6, 1);

    // localizing module-variables
    // state variables
    Matrix DXD(dxd);
    Matrix DX(dx);
    Matrix DDXD(ddxd);
    Matrix DDX(ddx);
    Matrix DYD(dyd);
    Matrix DY(dy);
    Matrix DDYD(ddyd);
    Matrix DDY(ddy);
    // packing state variables
    for (int n = 0; n < 3; n++) {
        DZD[n] = DXD[n];
        DZD[n + 3] = DYD[n];
        DZ[n] = DX[n];
        DZ[n + 3] = DY[n];
        DDZD[n] = DDXD[n];
        DDZD[n + 3] = DDYD[n];
        DDZ[n] = DDX[n];
        DDZ[n + 3] = DDY[n];
    }
    //-------------------------------------------------------------------------
    // second order integrations for each surface
    for (int i = 0; i < num_fins; i++) {
        // limiting fin position, rate, and rate derivative
        if (DZ[i] > dlimx) {
            DZ[i] = dlimx;
            if (DZ[i] * DDZ[i] > 0)
                DDZ[i] = 0;
        }
        if (DZ[i] < dlimx_min) {
            DZ[i] = dlimx_min;
            if (DZ[i] * DDZ[i] > 0)
                DDZ[i] = 0;
        }
        // limiting fin rate
        int iflag = 0;
        if (fabs(DDZ[i]) > ddlimx) {
            iflag = 1;
            DDZ[i] = ddlimx * sign(DDZ[i]);
        }
        // state integration
        DZD_NEW[i] = DDZ[i];
        DZ[i] = integrate(DZD_NEW[i], DZD[i], DZ[i], int_step);
        DZD[i] = DZD_NEW[i];
        double edx = ACTCZ[i] - DZ[i];
        DDZD_NEW[i] = wnact * wnact * edx - 2 * zetact * wnact * DZD[i];
        DDZ[i] = integrate(DDZD_NEW[i], DDZD[i], DDZ[i], int_step);
        DDZD[i] = DDZD_NEW[i];

        // limiting fin position, rate, and rate derivative
        if (DZ[i] > dlimx) {
            DZ[i] = dlimx;
            if (DZ[i] * DDZ[i] > 0)
                DDZ[i] = 0;
        }
        if (DZ[i] < dlimx_min) {
            DZ[i] = dlimx_min;
            if (DZ[i] * DDZ[i] > 0)
                DDZ[i] = 0;
        }
        // limiting fin rate
        iflag = 0;
        if (fabs(DDZ[i]) > ddlimx) {
            iflag = 1;
            DDZ[i] = ddlimx * sign(DDZ[i]);
        }
        // setting fin rate derivative to zero if rate is limited
        if (iflag && DDZ[i] * DDZD[i] > 0)
            DDZD[i] = 0;
    }
    //-------------------------------------------------------------------------
    // loading module-variables
    // unpacking of state variables
    for (int n = 0; n < 3; n++) {
        DXD[n] = DZD[n];
        DYD[n] = DZD[n + 3];
        DX[n] = DZ[n];
        DY[n] = DZ[n + 3];
        DDXD[n] = DDZD[n];
        DDYD[n] = DDZD[n + 3];
        DDX[n] = DDZ[n];
        DDY[n] = DDZ[n + 3];
    }
    // state variables
    DXD.fill(dxd);
    DX.fill(dx);
    DDXD.fill(ddxd);
    DDX.fill(ddx);
    DYD.fill(dyd);
    DY.fill(dy);
    DDYD.fill(ddyd);
    DDY.fill(ddy);

    return ACTZ = DZ;
}
