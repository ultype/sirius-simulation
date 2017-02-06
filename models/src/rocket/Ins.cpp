#include <cassert>

#include "aux/utility_header.hh"

#include "rocket/Ins.hh"
#include "sim_services/include/simtime.h"

#include "sensor/gyro/gyro.hh"
#include "sensor/gyro/gyro_ideal.hh"
#include "sensor/gyro/gyro_rocket6g.hh"

INS::INS(Newton &ntn, _Euler_ &elr, Environment &env, Kinematics &kins, GPS_Receiver &gps)
    :   newton(&ntn), euler(&elr), environment(&env), kinematics(&kins), gpsr(&gps),
        MATRIX_INIT(TDCI, 3, 3),
        MATRIX_INIT(TBIC, 3, 3),
        VECTOR_INIT(RICI, 3),
        VECTOR_INIT(RICID, 3),
        VECTOR_INIT(ESBI, 3),
        VECTOR_INIT(ESBID, 3),
        VECTOR_INIT(EVBI, 3),
        VECTOR_INIT(EVBID, 3),
        VECTOR_INIT(EGRAVI, 3),
        VECTOR_INIT(VBECD, 3),
        VECTOR_INIT(SBIIC, 3),
        VECTOR_INIT(VBIIC, 3),
        VECTOR_INIT(WBICI, 3)
{
    this->default_data();
}

INS::INS(const INS& other)
    :   newton(other.newton), euler(other.euler), environment(other.environment), kinematics(other.kinematics), gpsr(other.gpsr),
        MATRIX_INIT(TDCI, 3, 3),
        MATRIX_INIT(TBIC, 3, 3),
        VECTOR_INIT(RICI, 3),
        VECTOR_INIT(RICID, 3),
        VECTOR_INIT(ESBI, 3),
        VECTOR_INIT(ESBID, 3),
        VECTOR_INIT(EVBI, 3),
        VECTOR_INIT(EVBID, 3),
        VECTOR_INIT(EGRAVI, 3),
        VECTOR_INIT(VBECD, 3),
        VECTOR_INIT(SBIIC, 3),
        VECTOR_INIT(VBIIC, 3),
        VECTOR_INIT(WBICI, 3)
{
    this->default_data();
}

INS& INS::operator=(const INS& other){
    if(&other == this)
        return *this;

    this->newton      = other.newton;
    this->euler       = other.euler;
    this->environment = other.environment;
    this->kinematics  = other.kinematics;
    this->gpsr        = other.gpsr;

    return *this;
}

void INS::default_data(){
    this->EGRAVI.zeros();
}

void INS::initialize(){
}

void INS::set_gyro(sensor::Gyro &gyro) { this->gyro = &gyro; }
void INS::set_accelerometer(sensor::Accelerometer &accel) { this->accel = &accel; }

sensor::Gyro& INS::get_gyro() { return *gyro; }
sensor::Accelerometer& INS::get_accelerometer() { return *accel; }

void INS::set_ideal(){

    ESBI.zeros();
    EVBI.zeros();
    RICI.zeros();

    return;
}

/* frax_algnmnt : Fractn to mod initial INS err state: XXO=XXO(1+frax) */
void INS::set_non_ideal(double frax_algnmnt){
    // Initial covariance matrix  (GPS quality)
    // equipped aircraft. Units: meter, meter/sec, milli-rad.
    arma::mat99 PP_INIT = {{20.701      , 0.12317     , 0.10541     , 6.3213E-02  , 2.2055E-03  , 1.7234E-03  , 1.0633E-03  , 3.4941E-02  , -3.5179E-02} ,
                           {0.12317     , 20.696      , -0.27174    , 4.8366E-03  , 5.9463E-02  , -1.3367E-03 , -3.4903E-02 , 2.6112E-03  , -4.2663E-02} ,
                           {0.10541     , -0.27174    , 114.12      , 5.6373E-04  , -8.3147E-03 , 5.4059E-02  , 1.5496E-02  , 7.6463E-02  , -3.5302E-03} ,
                           {6.3213E-02  , 4.8366E-03  , 5.6373E-04  , 1.9106E-03  , 8.0945E-05  , 1.9810E-06  , 2.5755E-04  , 2.8346E-03  , -5.6482E-04} ,
                           {2.2055E-03  , 5.9463E-02  , -8.3147E-03 , 8.0945E-05  , 1.7201E-03  , -1.5760E-05 , -2.8341E-03 , 2.6478E-04  , -1.0781E-03} ,
                           {1.7234E-03  , -1.3367E-03 , 5.4059E-02  , 1.9810E-06  , -1.5760E-05 , 3.0070E-03  , 4.1963E-04  , -1.3297E-04 , 4.1190E-05}  ,
                           {1.0638E-03  , -3.4903E-02 , 1.5496E-02  , 2.5755E-04  , -2.8341E-03 , 4.1963E-04  , 5.4490E-02  , -1.8695E-03 , 8.9868E-04}  ,
                           {3.4941E-02  , 2.6112E-03  , 7.6463E-02  , 2.8346E-03  , 2.6478E-04  , -1.3297E-04 , -1.8695E-03 , 5.2819E-02  , 1.0990E-02}  ,
                           {-3.5179E-02 , -4.2663E-02 , -3.5302E-03 , -5.6482E-04 , -1.0781E-03 , 4.1190E-05  , 8.9868E-04  , 1.0990E-02  , 0.1291}};

    // getting square root of covariance matrix
    arma::mat99 APP_INIT = arma::chol(PP_INIT);

    // drawing Gaussian 9x1 vector with unit std deviation
    arma::vec9 GAUSS_INIT(arma::fill::randn);

    // forming stochastic initial state vector
    arma::vec9 XX_INIT = APP_INIT * GAUSS_INIT;
    XX_INIT *= (1 + frax_algnmnt);

    // forming subvectors for initialization and converting tilt to radians
    ESBI = XX_INIT.subvec(0, 2);
    EVBI = XX_INIT.subvec(3, 5);
    // tilt converted from milliradians to radians
    RICI = XX_INIT.subvec(6, 8) * 0.001;

    return;
}

arma::vec3 INS::calculate_gravity_error(double dbi){
    double dbic = norm(SBIIC);
    double ed = dbic - dbi;
    double dum = GM / pow(dbic, 3);
    if (dbic != 0) {
        return ESBI * (-dum) - SBIIC * (3 * ed * dum / dbic);
    } else {
        return arma::vec3(arma::fill::zeros);
    }
}

void INS::update(double int_step){
    assert(gyro && "INS module must be given a gyro model");
    assert(accel && "INS module must be given a accelerometer model");

    gyro->propagate_error(int_step);
    accel->propagate_error(int_step);

    // local variables
    double lonc(0), latc(0);
    double phipc(0);
    double psivdc(0), thtvdc(0);
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    // input from other modules
    double time      = get_rettime();
    arma::vec3 GRAVG = environment->get_GRAVG_();
    arma::mat33 TBI  = kinematics->get_TBI_();
    arma::vec3 WBIB  = euler->get_WBIB_();
    arma::vec3 WBII  = euler->get_WBII_();
    arma::vec3 SBII  = newton->get_SBII();
    arma::vec3 VBII  = newton->get_VBII();
    arma::vec3 FSPB  = newton->get_FSPB_();

    int mroll = 0; // Ambiguous

    arma::vec3 SXH(gpsr->position_state);
    arma::vec3 VXH(gpsr->velocity_state);

    arma::vec3 WBICB;
    arma::vec3 EWBIB;
    arma::vec3 FSPCB;
    arma::vec3 EFSPB;

    // computing INS derived postion of hyper B wrt center of Earth I
    SBIIC = ESBI + SBII;
    dbic = norm(SBIIC);

    // Gyro Measurement
    WBICB = gyro->get_computed_WBIB();

    // computed transformation matrix
    arma::mat33 UNI(arma::fill::eye);
    arma::mat33 TIIC = UNI - skew_sym(RICI);
    TBIC = TBI * TIIC;

    // Accelerometer Measurement
    FSPCB = accel->get_computed_FSPB();
    EFSPB = accel->get_error_of_computed_FSPB();

    // calculate gravitational error
    this->EGRAVI = calculate_gravity_error(newton->get_dbi());

    // integrating velocity error equation
    // XXX: How come INS integrates the Errorous Velocity based on EFSPB?
    arma::mat33 TICB = trans(TBIC);

    INTEGRATE_D(EVBI, TICB * EFSPB - skew_sym(RICI) * TICB * FSPCB + EGRAVI);

    // calculating position error
    INTEGRATE_D(ESBI, EVBI);

    // GPS update
    if (gpsr->gps_update) {
        // updating INS navigation output
        SBIIC = SBIIC - SXH;
        VBIIC = VBIIC - VXH;
        // resetting INS error states
        ESBI = ESBI - SXH;
        EVBI = EVBI - VXH;
        // returning flag to GPS that update was completed
        gpsr->gps_update--;
    }

    // computing INS derived position of hyper B wrt center of Earth I
    SBIIC = ESBI + SBII;
    // computing INS derived velocity of hyper B wrt inertial frame I
    VBIIC = EVBI + VBII;
    // computing INS derived body rates in inertial coordinates
    WBICI = trans(TBIC) * WBICB;

    // diagnostics
    ins_pos_err  = norm(ESBI);
    ins_vel_err  = norm(EVBI);
    ins_tilt_err = norm(RICI);

    // computing geographic velocity in body coordinates from INS
    arma::vec3 VEIC;
    VEIC(0) = -WEII3 * SBIIC(1);
    VEIC(1) =  WEII3 * SBIIC(0);
    VEIC(2) =  0;
    arma::vec3 VBEIC = VBIIC - VEIC;
    arma::vec3 VBECB = TBIC * VBEIC;
    dvbec = norm(VBECB);

    // decomposing computed body rates
    ppcx = WBICB(0) * DEG;
    qqcx = WBICB(1) * DEG;
    rrcx = WBICB(2) * DEG;

    // computing indidence angles from INS
    double alphac = atan2(VBECB(2), VBECB(0));
    double betac = asin(VBECB(1) / dvbec);
    alphacx = alphac * DEG;
    betacx = betac * DEG;

    // incidence angles in load factor plane (aeroballistic)
    double dum = VBECB(0) / dvbec;
    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alppc = acos(dum);

    if (VBECB(1) == 0 && VBECB(2) == 0)
        phipc = 0.;
    // note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
    //       to prevent oscillations
    else if (fabs(VBECB(1)) < EPS)
        if (VBECB(2) > 0)
            phipc = 0;
    if (VBECB(2) < 0)
        phipc = PI;
    else
        phipc = atan2(VBECB(1), VBECB(2));
    alppcx = alppc * DEG;
    phipcx = phipc * DEG;

    // getting long,lat,alt from INS
    arma_cad_geo84_in(lonc, latc, altc, SBIIC, time);

    // getting T.M. of geodetic wrt inertial coord
    TDCI = arma_cad_tdi84(lonc, latc, altc, time);
    loncx = lonc * DEG;
    latcx = latc * DEG;

    // computing geodetic velocity from INS
    VBECD = TDCI * VBEIC;

    // computing flight path angles
    if (VBECD[0] == 0 && VBECD[1] == 0) {
        psivdc = 0;
        thtvdc = 0;
    } else {
        psivdc = atan2(VBECD[1], VBECD[0]);
        thtvdc =
            atan2(-VBECD[2], sqrt(VBECD[0] * VBECD[0] + VBECD[1] * VBECD[1]));
    }
    psivdcx = psivdc * DEG;
    thtvdcx = thtvdc * DEG;

    // computing Euler angles from INS
    arma::mat33 TBD = TBIC * trans(TDCI);
    double tbd13 = TBD(0, 2);
    double tbd11 = TBD(0, 0);
    double tbd33 = TBD(2, 2);
    double tbd12 = TBD(0, 1);
    double tbd23 = TBD(1, 2);

    //*geodetic Euler angles
    // computed pitch angle: 'thtbdc'
    // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
    // forced to be a small positive number to prevent division by zero
    if (fabs(tbd13) < 1) {
        thtbdc = asin(-tbd13);
        cthtbd = cos(thtbdc);
    } else {
        thtbdc = PI / 2 * sign(-tbd13);
        cthtbd = EPS;
    }
    // computed yaw angle: 'psibdc'
    double cpsi = tbd11 / cthtbd;
    if (fabs(cpsi) > 1)
        cpsi = 1 * sign(cpsi);
    psibdc = acos(cpsi) * sign(tbd12);

    // computed roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if (fabs(cphi) > 1)
        cphi = 1 * sign(cphi);

    // selecting the Euler roll angle of flight mechanics (not for thtbdc=90 or
    // =-90deg)
    if (mroll == 0 || mroll == 1)
        // roll feedback for right side up
        phibdc = acos(cphi) * sign(tbd23);
    else if (mroll == 2)
        // roll feedback for inverted flight
        phibdc = acos(-cphi) * sign(-tbd23);

    psibdcx = DEG * psibdc;
    thtbdcx = DEG * thtbdc;
    phibdcx = DEG * phibdc;
}

double INS::get_dvbec() { return dvbec; }
double INS::get_qqcx() { return qqcx; }
double INS::get_rrcx() { return rrcx; }
double INS::get_ppcx() { return ppcx; }
double INS::get_alphacx() { return alphacx; }
double INS::get_betacx() { return betacx; }
double INS::get_phibdcx() { return phibdcx; }
double INS::get_thtbdcx() { return thtbdcx; }
double INS::get_psibdcx() { return psibdcx; }

Matrix INS::get_SBIIC() {
    Matrix SBIIC(_SBIIC);
    return SBIIC;
}
Matrix INS::get_VBIIC() {
    Matrix VBIIC(_VBIIC);
    return VBIIC;
}
Matrix INS::get_WBICI() {
    Matrix WBICI(_WBICI);
    return WBICI;
}
Matrix INS::get_EGRAVI() {
    Matrix EGRAVI(_EGRAVI);
    return EGRAVI;
}

Matrix INS::get_TBIC() {
    Matrix TBIC(3, 3);
    TBIC.build_mat33(_TBIC);
    return ~TBIC;
}
