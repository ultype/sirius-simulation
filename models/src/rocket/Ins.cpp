#include <cassert>

#include "cad/utility.hh"

#include "math/utility.hh"
#include "math/integrate.hh"
#include "math/matrix/utility.hh"

#include "rocket/Ins.hh"
#include "sim_services/include/simtime.h"

#include "sensor/gyro/gyro.hh"
#include "sensor/gyro/gyro_ideal.hh"
#include "sensor/gyro/gyro_rocket6g.hh"

#include "sensor/accel/accelerometer.hh"
#include "sensor/accel/accelerometer_ideal.hh"
#include "sensor/accel/accelerometer_rocket6g.hh"

#include <fstream>

INS::INS(time_management &tim)
    :   time(&tim),
        MATRIX_INIT(WEII, 3, 3),
        MATRIX_INIT(TDCI, 3, 3),
        MATRIX_INIT(TBIC, 3, 3),
        MATRIX_INIT(TEIC, 3, 3),
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
        VECTOR_INIT(SBEEC, 3),
        VECTOR_INIT(VBEEC, 3),
        VECTOR_INIT(WBICI, 3),
        VECTOR_INIT(TBIC_Q, 4),
        VECTOR_INIT(TBIDC_Q, 4)
{
    this->default_data();
}

INS::INS(const INS& other)
    :   time(other.time),
        MATRIX_INIT(WEII, 3, 3),
        MATRIX_INIT(TDCI, 3, 3),
        MATRIX_INIT(TBIC, 3, 3),
        MATRIX_INIT(TEIC, 3, 3),
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
        VECTOR_INIT(SBEEC, 3),
        VECTOR_INIT(VBEEC, 3),
        VECTOR_INIT(WBICI, 3),
        VECTOR_INIT(TBIC_Q, 4),
        VECTOR_INIT(TBIDC_Q ,4)
{
    this->default_data();

    this->grab_SBII = other.grab_SBII;
    this->grab_VBII = other.grab_VBII;
    this->grab_dbi  = other.grab_dbi;
    this->grab_TBI  = other.grab_TBI;
    this->grab_SXH = other.grab_SXH;
    this->grab_VXH = other.grab_VXH;
    this->grab_gps_update = other.grab_gps_update;
    this->clear_gps_flag = other.clear_gps_flag;

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

    this->time = other.time;
    this->grab_SBII = other.grab_SBII;
    this->grab_VBII = other.grab_VBII;
    this->grab_dbi  = other.grab_dbi;
    this->grab_TBI  = other.grab_TBI;
    this->grab_SXH = other.grab_SXH;
    this->grab_VXH = other.grab_VXH;
    this->grab_gps_update = other.grab_gps_update;
    this->clear_gps_flag = other.clear_gps_flag;

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
    WEII(0,2) = WEII2;
    WEII(2,0) = -WEII2;
    WEII(0,1) = -WEII3;
    WEII(1,0) =  WEII3;
    WEII(1,2) = -WEII1;
    WEII(2,1) = WEII1;
    return WEII;
}

void INS::default_data(){
    this->WEII = build_WEII();

    this->EGRAVI.zeros();
}

void INS::initialize(time_management &ti){
    time = &ti;

    TEIC = calculate_INS_derived_TEI();

    SBEEC = TEIC * SBIIC;
    VBEEC = TEIC * VBIIC - WEII * SBEEC;
}

void INS::load_location(double lonx, double latx, double alt){
    this->loncx = lonx;
    this->latcx = latx;
    this->altc = alt;

    //converting geodetic lonx, latx, alt to SBII
    SBIIC = cad::in_geo84(loncx * RAD, latcx * RAD, altc, get_rettime());
}

void INS::load_angle(double yaw, double roll, double pitch) {
    this->psibdcx = yaw;
    this->phibdcx = roll;
    this->thtbdcx = pitch;

    arma::mat33 TBD;

    TBD = build_psi_tht_phi_TM(psibdcx * RAD, thtbdcx * RAD, phibdcx * RAD);

    arma::mat33 current_TDI = cad::tdi84(loncx * RAD, latcx * RAD, altc, get_rettime());
    TBIC = TBD * current_TDI;

    this->TBIC_Q = Matrix2Quaternion(this->TBIC);  //Convert Direct Cosine Matrix to Quaternion
}

void INS::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe){
    //building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    arma::mat33 TBD = build_psi_tht_phi_TM(psibdcx * RAD, thtbdcx * RAD, phibdcx * RAD);
    arma::mat33 TDI = cad::tdi84(loncx * RAD, latcx * RAD, altc, get_rettime());
    //Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;

    VBIIC = trans(TDI) * VBED + WEII * SBIIC;
}

arma::vec INS::build_VBEB(double _alpha0x, double _beta0x, double _dvbe){
    double salp = sin(_alpha0x * RAD);
    double calp = cos(_alpha0x * RAD);
    double sbet = sin(_beta0x * RAD);
    double cbet = cos(_beta0x * RAD);
    double vbeb1 = calp * cbet * _dvbe;
    double vbeb2 = sbet * _dvbe;
    double vbeb3 = salp * cbet * _dvbe;
    arma::vec3 VBEB= {vbeb1, vbeb2, vbeb3};
    return VBEB;
}

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

arma::mat33 INS::calculate_INS_derived_TEI(){

    arma::mat33 TEIC;
    /* double We = 7.2921151467E-5; */
    //GPSR gpsr;/* call gpsr function */
    CALDATE utc_caldate;
    GPS tmp_gps;
    unsigned char  i;
    double UTC, UT1;
    arma::mat33 M_rotation;
    arma::mat33 M_nutation; M_nutation.eye();
    arma::mat33 M_precession;
    arma::mat33 M_nut_n_pre;
    double t, t2, t3, thetaA, zetaA, zA;
    double epsilonA, epsilonAP, F,D,omega;
    double temps_sideral(0);
    double L, La, gamma, delta_psi, delta_epsilon;
    double dUT1;
    double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;
    double s_delta_psi, c_delta_psi, s_epsilonA, c_epsilonA, s_epsilonAP, c_epsilonAP;
    double s2_half_delta_psi, s_delta_epsilon, c_delta_epsilon;
    double DM_sidereal_time;
    double DM_Julian_century;
    double DM_w_precessing;
    double mjd;/*double GC_swtwopi;*/

    int index;





    /*------------------------------------------------------------------ */
    /* --------------- Interface to Global Variable ------------*/
    /*------------------------------------------------------------------ */

    /*------------------------------------------------------------- */
    /*--------------- Calculate the UTC time -------------*/
    /*------------------------------------------------------------- */
    /* GPS time converted from GPS format to YYYY/MM/DD/MM/SS */
    /* Correction for time difference btwn GPS & UTC is applied implicitly */
    /***to prevent change origin gpstime data****/
    tmp_gps.Week  = time->gpstime.Week;
    tmp_gps.SOW  = time->gpstime.SOW;
    /*********************************************/
    time->gps_to_utc(&tmp_gps, &utc_caldate);   /* leap second is considered */

    UTC = utc_caldate.Hour * 3600.0 + utc_caldate.Min * 60.0 + utc_caldate.Sec;


    index = (int) (time->Julian_Date - 2400000.5 - 55197.00);   /* get dUT1 = Ut1 - UT from table*/
    if ((index >= 0) && (index < Max_DM_UT1_UT_Index))      /*MJD = 55197.00 (1-1-2010)~ 56150.00(8-11-2012) */
    {
        dUT1 = DM_UT1_UT[index];
    }
    else
    {
        dUT1 = -0.008853655954360;  /* mean value during 19760519~20120811, FSW: dUT1 = 0.4; */
    }

    UT1 = UTC + dUT1;



    /*----------------------------------------------------------- */
    /*-------------- Precession Matrix  ------------------- */
    /*----------------------------------------------------------- */
    t = (time->Julian_Date - 2451545.0) / 36525.0;  /* J2000.5 : Julian Day is 2451545, unit in day */

    DM_Julian_century = t;  /* elapsed century since J2000.5 */

    t2 = t * t;
    t3 = t * t * t;

    thetaA = 2004.3109 * t - 0.42665 * t2 - 0.041833 * t3; /* unit : arcsec */
    zetaA  = 2306.2181 * t + 0.30188 * t2 + 0.017998 * t3; /* unit : arcsec */
    zA     = 2306.2181 * t + 1.09468 * t2 + 0.018203 * t3; /* unit : arcsec */

    s_thetaA = sin(thetaA * DM_arcsec2r);
    c_thetaA = cos(thetaA * DM_arcsec2r);
    s_zetaA  = sin(zetaA  * DM_arcsec2r);
    c_zetaA  = cos(zetaA  * DM_arcsec2r);
    s_zA     = sin(zA * DM_arcsec2r);
    c_zA     = cos(zA * DM_arcsec2r);

    M_precession(0,0) = -s_zA * s_zetaA + c_zA * c_thetaA * c_zetaA;
    M_precession(0,1) = -s_zA * c_zetaA - c_zA * c_thetaA * s_zetaA;
    M_precession(0,2) = -c_zA * s_thetaA;
    M_precession(1,0) =  c_zA * s_zetaA + s_zA * c_thetaA * c_zetaA;
    M_precession(1,1) =  c_zA * c_zetaA - s_zA * c_thetaA * s_zetaA;
    M_precession(1,2) = -s_zA * s_thetaA;
    M_precession(2,0) =  s_thetaA * c_zetaA;
    M_precession(2,1) = -s_zetaA  * s_thetaA;
    M_precession(2,2) =  c_thetaA;


    M_nut_n_pre = M_nutation * M_precession;

    /*----------------------------------------------------------- */
    /*------------------- Rotation Matrix --------------------*/
    /*----------------------------------------------------------- */



    DM_w_precessing = 7.2921158553e-5 + 4.3e-15 * t; /* refer to Vallado */



    temps_sideral = UT1 +  (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3);


    temps_sideral = temps_sideral * DM_sec2r + delta_psi * cos(epsilonA) * DM_arcsec2r; /* unit: radian */


    DM_sidereal_time = temps_sideral;



    M_rotation(0,0) = cos(temps_sideral);
    M_rotation(0,1) = sin(temps_sideral);
    M_rotation(0,2) = 0.0;
    M_rotation(1,0) = -sin(temps_sideral);
    M_rotation(1,1) = cos(temps_sideral);
    M_rotation(1,2) = 0.0;
    M_rotation(2,0) = 0.0;
    M_rotation(2,1) = 0.0;
    M_rotation(2,2) = 1.0;


    TEIC = M_rotation * M_nut_n_pre;

    return TEIC;

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
    if (grab_gps_update()) {
        // GPS Measurement
        arma::vec3 SXH = grab_SXH();
        arma::vec3 VXH = grab_VXH();

        // updating INS navigation output
        SBIIC = SBIIC - SXH;
        VBIIC = VBIIC - VXH;
        // resetting INS error states
        ESBI = ESBI - SXH;
        EVBI = EVBI - VXH;
        // returning flag to GPS that update was completed
        clear_gps_flag();

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
    // local variables
    double lonc(0), latc(0);
    double psivdc(0), thtvdc(0);
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    // input from other modules
    double time      = get_rettime();
    arma::vec3 SBII  = grab_SBII();
    arma::vec3 VBII  = grab_VBII();
    arma::mat33 TBI  = grab_TBI();

    int mroll = 0; // Ambiguous

    // Gyro Measurement
    arma::vec3 WBICB = grab_computed_WBIB();
    arma::vec3 EWBIB = grab_error_of_computed_WBIB();

    // Accelerometer Measurement
    arma::vec3 FSPCB = grab_computed_FSPB();
    arma::vec3 EFSPB = grab_error_of_computed_FSPB();

    this->SBIIC = calculate_INS_derived_postion(SBII); //  wrt center of Earth I
    //dbic = norm(SBIIC);

    /* INS Tile Error Propagation */
    INTEGRATE_MAT(RICI, TBI * EWBIB);

    // computed transformation matrix
    // this->TBIC = calculate_INS_derived_TBI(TBI);
    propagate_TBI_Q(int_step, WBICB);



    // calculate gravitational error
    this->EGRAVI = calculate_gravity_error(grab_dbi());

    // integrating velocity error equation
    // XXX: How come INS integrates the Errorous Velocity based on EFSPB?
    arma::mat33 TICB = trans(TBIC);
    INTEGRATE_MAT(EVBI, TICB * EFSPB - skew_sym(RICI) * TICB * FSPCB + EGRAVI);

    // calculating position error
    INTEGRATE_MAT(ESBI, EVBI);

    if(gpsupdate == 1){
        GPS_update();
    }

    this->SBIIC = calculate_INS_derived_postion(SBII);
    this->VBIIC = calculate_INS_derived_velocity(VBII);
    this->WBICI = calculate_INS_derived_bodyrate(TBIC, WBICB);

    // diagnostics
    ins_pos_err  = norm(SBII - SBIIC);//norm(ESBI);
    ins_vel_err  = norm(VBII - VBIIC);//norm(EVBI);
    ins_tilt_err = norm(RICI);

    TEIC = calculate_INS_derived_TEI();
    // arma::mat33 TEI;
    // TEI = grab_TEI();

    SBEEC = TEIC * SBIIC;
    VBEEC = TEIC * VBIIC - WEII * SBEEC;

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
double INS::get_thtvdcx() { return thtvdcx; }

arma::vec3 INS::get_SBIIC() { return SBIIC; }
arma::vec3 INS::get_VBIIC() { return VBIIC; }
arma::vec3 INS::get_SBEEC() { return SBEEC; }
arma::vec3 INS::get_VBEEC() { return VBEEC; }
arma::vec3 INS::get_WBICI() { return WBICI; }
arma::vec3 INS::get_EGRAVI() { return EGRAVI; }
arma::mat33 INS::get_TBIC() { return TBIC; }
arma::mat33 INS::get_TEIC() { return TEIC; }

void INS::set_gps_correction(unsigned int index){ gpsupdate = index; }

void INS::propagate_TBI_Q(double int_step, arma::vec3 WBICB){
    arma::vec TBIDC_Q_NEW(4);
    /* Prepare for orthonormalization */
    double quat_metric = TBIC_Q(0) * TBIC_Q(0) + TBIC_Q(1) * TBIC_Q(1) + TBIC_Q(2) * TBIC_Q(2) + TBIC_Q(3) * TBIC_Q(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */
    TBIDC_Q_NEW(0) = 0.5 * (-WBICB(0) * TBIC_Q(1) - WBICB(1) * TBIC_Q(2) - WBICB(2) * TBIC_Q(3)) + 50. * erq * TBIC_Q(0);
    TBIDC_Q_NEW(1) = 0.5 * (WBICB(0) * TBIC_Q(0) + WBICB(2) * TBIC_Q(2) - WBICB(1) * TBIC_Q(3)) + 50. * erq * TBIC_Q(1);
    TBIDC_Q_NEW(2) = 0.5 * (WBICB(1) * TBIC_Q(0) - WBICB(2) * TBIC_Q(1) + WBICB(0) * TBIC_Q(3)) + 50. * erq * TBIC_Q(2);
    TBIDC_Q_NEW(3) = 0.5 * (WBICB(2) * TBIC_Q(0) + WBICB(1) * TBIC_Q(1) - WBICB(0) * TBIC_Q(2)) + 50. * erq * TBIC_Q(3);

    this->TBIC_Q = integrate(TBIDC_Q_NEW, this->TBIDC_Q, this->TBIC_Q, int_step);

    this->TBIDC_Q = TBIDC_Q_NEW;

    this->TBIC = Quaternion2Matrix(this->TBIC_Q);  //Convert Quaternion to Matrix

    //TBI orthogonality check
    // arma::mat TIBC = trans(TBIC);
    // arma::mat UBI = TIBC * TBIC;
    // double e1 = UBI(0,0) - 1.;
    // double e2 = UBI(1,1) - 1.;
    // double e3 = UBI(2,2) - 1.;
    // this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}
