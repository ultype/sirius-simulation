#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

#include "aux/global_constants.hh"
#include "math/stochastic.hh"
#include "math/matrix/utility.hh"

#include "cad/utility.hh"

#include <armadillo>

#include "rocket/Newton.hh"
#include "rocket/Euler.hh"
#include "rocket/Ins.hh"
GPS::GPS(time_management &time_ma, GPS_constellation &gps_cons)
:   time(&time_ma),
    gps_con(&gps_cons),
    MATRIX_INIT(PP, 8, 8),
    MATRIX_INIT(FF, 8, 8),
    MATRIX_INIT(PHI, 8, 8),
    VECTOR_INIT(SXH, 3),
    VECTOR_INIT(VXH, 3),
    VECTOR_INIT(CXH, 3)
{}

GPS::GPS(const GPS &other)
:   time(other.time),
    gps_con(other.gps_con),
    MATRIX_INIT(PP, 8, 8),
    MATRIX_INIT(FF, 8, 8),
    MATRIX_INIT(PHI, 8, 8),
    VECTOR_INIT(SXH, 3),
    VECTOR_INIT(VXH, 3),
    VECTOR_INIT(CXH, 3)
{}

GPS & GPS::operator= (const GPS &other){
    if(&other == this)
    return *this;
    this->time = other.time;
    this->gps_con = other.gps_con;

    return *this;
}


void GPS::setup_state_covariance_matrix(double factp, double pclockb, double pclockf){
    PP = arma::mat88(arma::fill::zeros);
    for (int i = 0; i < 3; i++) {
        PP(i, i)         = pow(ppos * (1 + factp), 2);
        PP(i + 3, i + 3) = pow(pvel * (1 + factp), 2);
    }
    PP(6, 6) =  pow(pclockb * (1 + factp), 2);
    PP(7, 7) =  pow(pclockf * (1 + factp), 2);

    return;
}

void GPS::setup_error_covariance_matrix(double factq, double qclockb, double qclockf){
    this->factq = factq;
    this->qclockb = qclockb;
    this->qclockf = qclockf;

    return;
}

void GPS::setup_fundamental_dynamic_matrix(double uctime_cor){
    // fundamental dynamic matrix of filter - constant throughout
    FF = arma::mat88(arma::fill::zeros);
    FF(0, 3) = 1;
    FF(1, 4) = 1;
    FF(2, 5) = 1;
    FF(6, 7) = 1;
    FF(7, 7) = -1 / uctime_cor;

    return;
}




void GPS::filter_extrapolation(double int_step){
    arma::mat88 QQ(arma::fill::zeros);  // local

    //*** user-clock frequency and bias error growth between updates ***
    // integrating 'ucfreq_noise' Markov process to
    //  obtain user-clock bias error 'ucbias_error' (trapezoidal
    //  integration)
    // user-clock bias is updated at filter update epoch
    ucfreq_error = ucfreq_noise;
    ucbias_error = ucbias_error + (ucfreq_error + ucfreqm) * (int_step / 2);
    ucfreqm = ucfreq_error;

    //*** filter extrapolation ***
    // dynamic error covariance matrix
    for (int i = 0; i < 3; i++) {
        QQ(i, i)         = pow(qpos * (1 + factq), 2);
        QQ(i + 3, i + 3) = pow(qvel * (1 + factq), 2);
    }
    QQ(6, 6) = pow(qclockb * (1 + factq), 2);
    QQ(7, 7) = pow(qclockf * (1 + factq), 2);

    // covariance estimate extrapolation
    PP = PHI * (PP + QQ * (int_step / 2)) * trans(PHI) + QQ * (int_step / 2);

    // diagnostics: st. deviations of the diagonals of the covariance matrix
    std_pos = sqrt(PP(0, 0));
    std_vel = sqrt(PP(3, 3));
    std_ucbias = sqrt(PP(6, 6));
}

void GPS::measure(){
    double dtime_gps;
    /* Testing GPS timing for update and acquire */
    if(!gps_acq)
        // saving delay-time for GPS signal acquisition
        dtime_gps = gps_acqtime;
    else
        // saving delay-time for GPS update
        dtime_gps = gps_step;
    // checking when GPS update time has occured in order to initiate update
    time_gps = get_rettime() - gps_epoch;
    if (time_gps < dtime_gps) {
        return;
    }

    gps_acq = true;
    gps_update++;
    // resetting update clock
    time_gps = 0;
    gps_epoch = get_rettime();

    arma::vec8 ZZ(arma::fill::zeros);
    arma::vec8 XH(arma::fill::zeros);           // local
    arma::mat88 RR(arma::fill::zeros);          // local
    arma::mat88 HH(arma::fill::zeros);          // local

    // ***
    // SV propagation and quadriga selection 'ssii_quad'
    // (4 SVs with best GDOP)
    // ***
    // gps_quadriga(ssii_quad,vsii_quad,gdop,mgps,
    // sv_init_data,rsi,wsi,incl,almanac_time,del_rearth,time,SBII);

    // Pseudo-range and range-rate measurements
    for (int i = 0; i < 4; i++) {

        // INS derived range measurements
        arma::vec3 SSBIC;
        SSBIC = SSII - SBIIC;
        double dsbc = norm(SSBIC);

        // INS derived range-rate measurements
        // velocity of SV wrt user
        arma::vec3 VSBIC;
        VSBIC = VSII - VBIIC - skew_sym(WBICI) * SSBIC;
        // calculating range-rate to SV
        arma::vec3 USSBIC;
        USSBIC = SSBIC * (1 / dsb);
        double dvsbc = dot(VSBIC, USSBIC);

        // loading measurement residuals into measurement vector
        // ZZ[0->3] range meas resid of SV's;
        // ZZ[4->7] range-rate meas resid of SV's
        ZZ[i] = dsb_meas - dsbc;
        ZZ[i + 4] = dvsb_meas - dvsbc;

        // observation matrix of filter
        for (int j = 0; j < 3; j++) {
            HH(i, j) = USSBI(j, 0);
            HH(i + 4, j + 3) = USSBI(j, 0) * gps_step;
        }
        HH(i, 6) = 1;
        HH(i + 4, 7) = gps_step;

    }
   
    //*** filter correction and update (to INS: 'SXH' and 'VXH') ***
    // filter gain
    arma::mat88 KK(arma::fill::zeros);

    // measurement noise covariance matrix
    for (int i = 0; i < 4; i++) {
        RR(i, i) = pow(rpos * (1 + factr), 2);
        RR(i + 4, i + 4) = pow(rvel * (1 + factr), 2);
    }
    // Kalman gain
    KK = arma::inv(PP * arma::trans(HH) * (HH * PP * arma::trans(HH) + RR));
    // state correction
    XH = KK * ZZ;
    // covariance correction for next cycle
    PP = (arma::mat88(arma::fill::eye) - KK * HH) * PP;

    // clock error bias update
    ucbias_error = ucbias_error - XH(6, 0);

    // diagnostics of 1st SV of quadriga saved to plot file
    gps_pos_meas = ZZ(0, 0);
    gps_vel_meas = ZZ(4, 0);

    // decomposing state vector for output
    for (int m = 0; m < 3; m++) {
        SXH(m, 0) = XH(m, 0);
        VXH(m, 0) = XH(m + 3, 0);
    }
    CXH(0, 0) = XH(6, 0);
    CXH(1, 0) = XH(7, 0);

    // diagnostic
    state_pos = norm(SXH);
    state_vel = norm(VXH);
}