#include <cassert>

#include "cad/utility.hh"

#include "math/utility.hh"
#include "math/integrate.hh"
#include "math/matrix/utility.hh"

#include "aux/utility_header.hh"

#include "rocket/Ins.hh"
#include "sim_services/include/simtime.h"

#include "sensor/gyro/gyro.hh"
#include "sensor/gyro/gyro_ideal.hh"
#include "sensor/gyro/gyro_rocket6g.hh"

#include "sensor/accel/accelerometer.hh"
#include "sensor/accel/accelerometer_ideal.hh"
#include "sensor/accel/accelerometer_rocket6g.hh"

#include <boost/serialization/export.hpp>

#include <fstream>

template<class Archive>
void INS::serialize(Archive & ar, const unsigned int version){
    ar.template register_type<sensor::GyroIdeal>();
    ar.template register_type<sensor::GyroRocket6G>();

    ar.template register_type<sensor::AccelerometerIdeal>();
    ar.template register_type<sensor::AccelerometerRocket6G>();

    ar & gyro;
    ar & accel;
    ar & _EVBI;
    ar & _EVBID;
    ar & _ESBI;
    ar & _ESBID;
    ar & _RICI;
    ar & _RICID;
    ar & _TBIC;
    ar & _SBIIC;
    ar & _VBIIC;
    ar & _WBICI;
    ar & _EGRAVI;
    ar & loncx;
    ar & latcx;
    ar & altc;
    ar & _VBECD;
    ar & _TDCI;
    ar & dbic;
    ar & dvbec;
    ar & alphacx;
    ar & betacx;
    ar & thtvdcx;
    ar & psivdcx;
    ar & alppcx;
    ar & phipcx;
    ar & phibdcx;
    ar & thtbdcx;
    ar & psibdcx;
}

INS::INS(Newton &ntn, _Euler_ &elr, Environment &env, Kinematics &kins, GPS_Receiver &gps)
    :   newton(&ntn), euler(&elr), environment(&env), kinematics(&kins), gpsr(&gps),
        MATRIX_INIT(WEII, 3, 3),
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
        MATRIX_INIT(WEII, 3, 3),
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

    this->gyro = new sensor::Gyro(*other.gyro);
    this->accel = new sensor::Accelerometer(*other.accel);

    /* Propagative Stats */
    this->EVBI = other.EVBI;
    this->EVBID = other.EVBID;
    this->ESBI = other.ESBI;
    this->ESBID = other.ESBID;
    this->RICI = other.RICI;
    this->RICID = other.RICID;

    /* Generating Outputs */
    this->TBIC = other.TBIC;
    this->SBIIC = other.SBIIC;
    this->VBIIC = other.VBIIC;
    this->WBICI = other.WBICI;
    this->EGRAVI = other.EGRAVI;

    this->loncx = other.loncx;
    this->latcx = other.latcx;
    this->altc = other.altc;

    this->VBECD = other.VBECD;
    this->TDCI = other.TDCI;

    this->dbic = other.dbic;
    this->dvbec = other.dvbec;

    this->alphacx = other.alphacx;
    this->betacx = other.betacx;

    this->thtvdcx = other.thtvdcx;
    this->psivdcx = other.psivdcx;

    this->alppcx = other.alppcx;
    this->phipcx = other.phipcx;

    this->phibdcx = other.phibdcx;
    this->thtbdcx = other.thtbdcx;
    this->psibdcx = other.psibdcx;
}

INS& INS::operator=(const INS& other){
    if(&other == this)
        return *this;

    this->newton      = other.newton;
    this->euler       = other.euler;
    this->environment = other.environment;
    this->kinematics  = other.kinematics;
    this->gpsr        = other.gpsr;

    this->gyro = new sensor::Gyro(*other.gyro);
    this->accel = new sensor::Accelerometer(*other.accel);

    /* Propagative Stats */
    this->EVBI = other.EVBI;
    this->EVBID = other.EVBID;
    this->ESBI = other.ESBI;
    this->ESBID = other.ESBID;
    this->RICI = other.RICI;
    this->RICID = other.RICID;

    /* Generating Outputs */
    this->TBIC = other.TBIC;
    this->SBIIC = other.SBIIC;
    this->VBIIC = other.VBIIC;
    this->WBICI = other.WBICI;
    this->EGRAVI = other.EGRAVI;

    this->loncx = other.loncx;
    this->latcx = other.latcx;
    this->altc = other.altc;

    this->VBECD = other.VBECD;
    this->TDCI = other.TDCI;

    this->dbic = other.dbic;
    this->dvbec = other.dvbec;

    this->alphacx = other.alphacx;
    this->betacx = other.betacx;

    this->thtvdcx = other.thtvdcx;
    this->psivdcx = other.psivdcx;

    this->alppcx = other.alppcx;
    this->phipcx = other.phipcx;

    this->phibdcx = other.phibdcx;
    this->thtbdcx = other.thtbdcx;
    this->psibdcx = other.psibdcx;

    return *this;
}

arma::mat INS::build_WEII(){
    arma::mat33 WEII;
    WEII.zeros();
    WEII(0, 1) = -WEII3;
    WEII(1, 0) =  WEII3;
    return WEII;
}

void INS::default_data(){
    this->WEII = build_WEII();

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

arma::vec3 INS::calculate_INS_derived_postion(arma::vec3 SBII){
    // computing INS derived postion of hyper B wrt center of Earth I
    return ESBI + SBII;
}

arma::vec3 INS::calculate_INS_derived_velocity(arma::vec3 VBII){
    // computing INS derived velocity of hyper B wrt inertial frame I
    return EVBI + VBII;
}

arma::vec3 INS::calculate_INS_derived_bodyrate(arma::mat33 TBIC, arma::vec3 WBICB){
    // computing INS derived body rates in inertial coordinates
    return trans(TBIC) * WBICB;
}

arma::mat33 INS::calculate_INS_derived_TBI(arma::mat33 TBI){
    // computed transformation matrix
    arma::mat33 UNI(arma::fill::eye);
    arma::mat33 TIIC = UNI - skew_sym(RICI);
    return TBI * TIIC;
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

double INS::calculate_INS_derived_dvbe(){
    return 0;
}

bool INS::GPS_update(){
    // GPS update
    if (gpsr->gps_update) {
        // GPS Measurement
        arma::vec3 SXH = gpsr->get_SXH();
        arma::vec3 VXH = gpsr->get_VXH();

        // updating INS navigation output
        SBIIC = SBIIC - SXH;
        VBIIC = VBIIC - VXH;
        // resetting INS error states
        ESBI = ESBI - SXH;
        EVBI = EVBI - VXH;
        // returning flag to GPS that update was completed
        gpsr->gps_update--;

        return true;
    }
    return false;
}

double INS::calculate_INS_derived_alpha(arma::vec3 VBECB){
    return atan2(VBECB(2), VBECB(0));
}

double INS::calculate_INS_derived_beta(arma::vec3 VBECB){
    return asin(VBECB(1) / norm(VBECB));
}

double INS::calculate_INS_derived_alpp(arma::vec3 VBECB){
    double dum = VBECB(0) / norm(VBECB);
    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    return acos(dum);
}

double INS::calculate_INS_derived_psivd(arma::vec3 VBECD){
    if (VBECD(0) == 0 && VBECD(1) == 0) {
        return 0;
    } else {
        return atan2(VBECD(1), VBECD(0));
    }
}

double INS::calculate_INS_derived_thtvd(arma::vec3 VBECD){
    if (VBECD(0) == 0 && VBECD(1) == 0) {
        return 0;
    } else {
        return atan2(-VBECD(2), sqrt(VBECD(0) * VBECD(0) + VBECD(1) * VBECD(1)));
    }

}

double INS::calculate_INS_derived_phip(arma::vec3 VBECB){
    double phipc(0);

    if (VBECB(1) == 0 && VBECB(2) == 0){

        phipc = 0.;
    } else if (fabs(VBECB(1)) < EPS) {
        // note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
        //       to prevent oscillations
        if (VBECB(2) > 0) phipc = 0;
        if (VBECB(2) < 0) phipc = PI;
    } else{
        phipc = atan2(VBECB(1), VBECB(2));
    }

    return phipc;
}

double INS::calculate_INS_derived_euler_angles(arma::mat33 TBD){
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    double mroll = 0;

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

    this->psibdcx = DEG * psibdc;
    this->thtbdcx = DEG * thtbdc;
    this->phibdcx = DEG * phibdc;

}

void INS::update(double int_step){
    assert(gyro && "INS module must be given a gyro model");
    assert(accel && "INS module must be given a accelerometer model");

    // local variables
    double lonc(0), latc(0);
    double psivdc(0), thtvdc(0);
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    // input from other modules
    double time      = get_rettime();
    arma::vec3 SBII  = newton->get_SBII();
    arma::vec3 VBII  = newton->get_VBII();
    arma::mat33 TBI  = kinematics->get_TBI();

    int mroll = 0; // Ambiguous

    // Gyro Measurement
    arma::vec3 WBICB = gyro->get_computed_WBIB();
    arma::vec3 EWBIB = gyro->get_error_of_computed_WBIB();

    // Accelerometer Measurement
    arma::vec3 FSPCB = accel->get_computed_FSPB();
    arma::vec3 EFSPB = accel->get_error_of_computed_FSPB();

    this->SBIIC = calculate_INS_derived_postion(SBII); //  wrt center of Earth I
    //dbic = norm(SBIIC);

    /* INS Tile Error Propagation */
    INTEGRATE_MAT(RICI, TBI * EWBIB);

    // computed transformation matrix
    this->TBIC = calculate_INS_derived_TBI(TBI);

    // calculate gravitational error
    this->EGRAVI = calculate_gravity_error(newton->get_dbi());

    // integrating velocity error equation
    // XXX: How come INS integrates the Errorous Velocity based on EFSPB?
    arma::mat33 TICB = trans(TBIC);
    INTEGRATE_MAT(EVBI, TICB * EFSPB - skew_sym(RICI) * TICB * FSPCB + EGRAVI);

    // calculating position error
    INTEGRATE_MAT(ESBI, EVBI);

    GPS_update();

    this->SBIIC = calculate_INS_derived_postion(SBII);
    this->VBIIC = calculate_INS_derived_velocity(VBII);
    this->WBICI = calculate_INS_derived_bodyrate(TBIC, WBICB);

    // diagnostics
    //ins_pos_err  = norm(ESBI);
    //ins_vel_err  = norm(EVBI);
    //ins_tilt_err = norm(RICI);

    // computing geographic velocity in body coordinates from INS
    arma::vec3 VEIC = WEII * SBIIC;
    arma::vec3 VBEIC = VBIIC - VEIC;
    arma::vec3 VBECB = TBIC * VBEIC;

    this->dvbec = norm(VBECB);

    // computing indidence angles from INS
    this->alphacx = calculate_INS_derived_alpha(VBECB) * DEG;
    this->betacx  = calculate_INS_derived_beta(VBECB) * DEG;

    // incidence angles in load factor plane (aeroballistic)
    this->alppcx = calculate_INS_derived_alpp(VBECB) * DEG;
    this->phipcx = calculate_INS_derived_phip(VBECB) * DEG;

    // getting long,lat,alt from INS
    cad::geo84_in(lonc, latc, altc, SBIIC, time);
    loncx = lonc * DEG;
    latcx = latc * DEG;

    // getting T.M. of geodetic wrt inertial coord
    this->TDCI = cad::tdi84(lonc, latc, altc, time);

    // computing geodetic velocity from INS
    arma::vec3 VBECD = TDCI * VBEIC;

    // computing flight path angles
    this->psivdcx = calculate_INS_derived_psivd(VBECD) * DEG;
    this->thtvdcx = calculate_INS_derived_thtvd(VBECD) * DEG;

    // computing Euler angles from INS
    arma::mat33 TBD = TBIC * trans(TDCI);
    calculate_INS_derived_euler_angles(TBD);
}

double INS::get_dvbec() { return dvbec; }
double INS::get_alphacx() { return alphacx; }
double INS::get_betacx() { return betacx; }
double INS::get_phibdcx() { return phibdcx; }
double INS::get_thtbdcx() { return thtbdcx; }
double INS::get_psibdcx() { return psibdcx; }

arma::vec3 INS::get_SBIIC() { return SBIIC; }
arma::vec3 INS::get_VBIIC() { return VBIIC; }
arma::vec3 INS::get_WBICI() { return WBICI; }
arma::vec3 INS::get_EGRAVI() { return EGRAVI; }
arma::mat33 INS::get_TBIC() { return TBIC; }
