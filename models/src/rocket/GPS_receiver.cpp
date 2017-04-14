#include "rocket/GPS_receiver.hh"
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

#include "rocket/GPS_satellites.hh"
#include "rocket/Newton.hh"
#include "rocket/Euler.hh"
#include "rocket/Earth.hh"
#include "fsw/Ins.hh"

GPS_Receiver::GPS_Receiver(Newton &ntn, _Euler_ &elr)
    :   newton(&ntn), euler(&elr),
        MATRIX_INIT(PP, 8, 8),
        MATRIX_INIT(FF, 8, 8),
        MATRIX_INIT(PHI, 8, 8),
        VECTOR_INIT(SXH, 3),
        VECTOR_INIT(VXH, 3),
        VECTOR_INIT(CXH, 3)
{
    this->default_data();
}

GPS_Receiver::GPS_Receiver(const GPS_Receiver& other)
    :   newton(other.newton), euler(other.euler), gps_sats(other.gps_sats),
        MATRIX_INIT(PP, 8, 8),
        MATRIX_INIT(FF, 8, 8),
        MATRIX_INIT(PHI, 8, 8),
        VECTOR_INIT(SXH, 3),
        VECTOR_INIT(VXH, 3),
        VECTOR_INIT(CXH, 3)
{
    this->default_data();

    /* Input File */
    memcpy(this->slot, other.slot, sizeof(this->slot));

    /* GPS Device parameters */
    this->del_rearth = other.del_rearth;
    this->gps_acqtime = other.gps_acqtime;
    this->gps_step = other.gps_step;

    this->ucfreq_noise = other.ucfreq_noise;
    this->ucfreq_noise_sigma = other.ucfreq_noise_sigma;
    this->ucfreq_noise_bcor = other.ucfreq_noise_bcor;
    this->ucbias_error = other.ucbias_error;

    memcpy(this->PR_BIAS        , other.PR_BIAS        , sizeof(this->PR_BIAS));
    memcpy(this->PR_NOISE       , other.PR_NOISE       , sizeof(this->PR_NOISE));
    memcpy(this->PR_NOISE_sigma , other.PR_NOISE_sigma , sizeof(this->PR_NOISE_sigma));
    memcpy(this->PR_NOISE_bcor  , other.PR_NOISE_bcor  , sizeof(this->PR_NOISE_bcor));
    memcpy(this->DR_NOISE       , other.DR_NOISE       , sizeof(this->DR_NOISE));
    memcpy(this->DR_NOISE_sigma , other.DR_NOISE_sigma , sizeof(this->DR_NOISE_sigma));
    memcpy(this->DR_NOISE_bcor  , other.DR_NOISE_bcor  , sizeof(this->DR_NOISE_bcor));

    /* GPS EKF Parameters */
    this->ppos = other.ppos;
    this->pvel = other.pvel;
    this->qpos = other.qpos;
    this->qvel = other.qvel;
    this->rpos = other.rpos;
    this->rvel = other.rvel;
    this->factr = other.factr;

    /* Internal variables */
    this->gps_acq = this->gps_acq;
    this->gps_epoch = other.gps_epoch;
    this->time_gps = other.time_gps;

    this->FF = other.FF;
    this->PHI = other.PHI;
    this->PP = other.PP;

    this->factq = other.factq;
    this->qclockb = other.qclockb;
    this->qclockf = other.qclockf;

    this->slotsum = other.slotsum;

    this->SXH = other.SXH;
    this->VXH = other.VXH;
    this->CXH = other.CXH;

    /* GPS Outputs */
    this->gdop = other.gdop;
    memcpy(this->ssii_quad, other.ssii_quad, sizeof(this->ssii_quad));
    memcpy(this->vsii_quad, other.vsii_quad, sizeof(this->vsii_quad));

    this->ucfreq_error = other.ucfreq_error;
    this->ucfreqm = other.ucfreqm;

    this->std_pos = other.std_pos;
    this->std_vel = other.std_vel;
    this->std_ucbias = other.std_ucbias;

    this->lat1 = other.lat1;
    this->lat2 = other.lat2;
    this->lat3 = other.lat3;
    this->lat4 = other.lat4;
    this->lon1 = other.lon1;
    this->lon2 = other.lon2;
    this->lon3 = other.lon3;
    this->lon4 = other.lon4;
    this->alt1 = other.alt1;
    this->alt2 = other.alt2;
    this->alt3 = other.alt3;
    this->alt4 = other.alt4;

    this->gps_pos_meas = other.gps_pos_meas;
    this->gps_vel_meas = other.gps_vel_meas;
    this->state_pos = other.state_pos;
    this->state_vel = other.state_vel;
}

GPS_Receiver& GPS_Receiver::operator=(const GPS_Receiver& other){
    if(&other == this)
        return *this;

    this->newton   = other.newton;
    this->euler    = other.euler;
    this->gps_sats = other.gps_sats;

    /* Input File */
    memcpy(this->slot, other.slot, sizeof(this->slot));

    /* GPS Device parameters */
    this->del_rearth = other.del_rearth;
    this->gps_acqtime = other.gps_acqtime;
    this->gps_step = other.gps_step;

    this->ucfreq_noise = other.ucfreq_noise;
    this->ucfreq_noise_sigma = other.ucfreq_noise_sigma;
    this->ucfreq_noise_bcor = other.ucfreq_noise_bcor;
    this->ucbias_error = other.ucbias_error;

    memcpy(this->PR_BIAS        , other.PR_BIAS        , sizeof(this->PR_BIAS));
    memcpy(this->PR_NOISE       , other.PR_NOISE       , sizeof(this->PR_NOISE));
    memcpy(this->PR_NOISE_sigma , other.PR_NOISE_sigma , sizeof(this->PR_NOISE_sigma));
    memcpy(this->PR_NOISE_bcor  , other.PR_NOISE_bcor  , sizeof(this->PR_NOISE_bcor));
    memcpy(this->DR_NOISE       , other.DR_NOISE       , sizeof(this->DR_NOISE));
    memcpy(this->DR_NOISE_sigma , other.DR_NOISE_sigma , sizeof(this->DR_NOISE_sigma));
    memcpy(this->DR_NOISE_bcor  , other.DR_NOISE_bcor  , sizeof(this->DR_NOISE_bcor));

    /* GPS EKF Parameters */
    this->ppos = other.ppos;
    this->pvel = other.pvel;
    this->qpos = other.qpos;
    this->qvel = other.qvel;
    this->rpos = other.rpos;
    this->rvel = other.rvel;
    this->factr = other.factr;

    /* Internal variables */
    this->gps_acq = this->gps_acq;
    this->gps_epoch = other.gps_epoch;
    this->time_gps = other.time_gps;

    this->FF = other.FF;
    this->PHI = other.PHI;
    this->PP = other.PP;

    this->factq = other.factq;
    this->qclockb = other.qclockb;
    this->qclockf = other.qclockf;

    this->slotsum = other.slotsum;

    this->SXH = other.SXH;
    this->VXH = other.VXH;
    this->CXH = other.CXH;

    /* GPS Outputs */
    this->gdop = other.gdop;
    memcpy(this->ssii_quad, other.ssii_quad, sizeof(this->ssii_quad));
    memcpy(this->vsii_quad, other.vsii_quad, sizeof(this->vsii_quad));

    this->ucfreq_error = other.ucfreq_error;
    this->ucfreqm = other.ucfreqm;

    this->std_pos = other.std_pos;
    this->std_vel = other.std_vel;
    this->std_ucbias = other.std_ucbias;

    this->lat1 = other.lat1;
    this->lat2 = other.lat2;
    this->lat3 = other.lat3;
    this->lat4 = other.lat4;
    this->lon1 = other.lon1;
    this->lon2 = other.lon2;
    this->lon3 = other.lon3;
    this->lon4 = other.lon4;
    this->alt1 = other.alt1;
    this->alt2 = other.alt2;
    this->alt3 = other.alt3;
    this->alt4 = other.alt4;

    this->gps_pos_meas = other.gps_pos_meas;
    this->gps_vel_meas = other.gps_vel_meas;
    this->state_pos = other.state_pos;
    this->state_vel = other.state_vel;

    return *this;
}

void GPS_Receiver::default_data(){
    gps_update = 0;

    // setting inital acquisition flag
    gps_acq = false;
}

void GPS_Receiver::setup_state_covariance_matrix(double factp, double pclockb, double pclockf){
    PP = arma::mat88(arma::fill::zeros);
    for (int i = 0; i < 3; i++) {
        PP(i, i)         = pow(ppos * (1 + factp), 2);
        PP(i + 3, i + 3) = pow(pvel * (1 + factp), 2);
    }
    PP(6, 6) =  pow(pclockb * (1 + factp), 2);
    PP(7, 7) =  pow(pclockf * (1 + factp), 2);

    return;
}

void GPS_Receiver::setup_error_covariance_matrix(double factq, double qclockb, double qclockf){
    this->factq = factq;
    this->qclockb = qclockb;
    this->qclockf = qclockf;

    return;
}

void GPS_Receiver::setup_fundamental_dynamic_matrix(double uctime_cor){
    // fundamental dynamic matrix of filter - constant throughout
    FF = arma::mat88(arma::fill::zeros);
    FF(0, 3) = 1;
    FF(1, 4) = 1;
    FF(2, 5) = 1;
    FF(6, 7) = 1;
    FF(7, 7) = -1 / uctime_cor;

    return;
}

void GPS_Receiver::initialize(GPS_Satellites *sats, double int_step){
    gps_sats = sats;

    // state transition matrix - constant throughout
    PHI = arma::mat88(arma::fill::eye) + FF * int_step + FF * FF * (int_step * int_step / 2);

    // initializing update clock
    gps_epoch = get_rettime();
}

void GPS_Receiver::update_markov(double int_step){
    int i;

    ucfreq_noise = markov(ucfreq_noise_sigma, ucfreq_noise_bcor, get_rettime(), int_step, ucfreq_noise);
    for(i = 0; i < 4; i++){
        PR_NOISE[i] = markov(PR_NOISE_sigma[i], PR_NOISE_bcor[i], get_rettime(), int_step, PR_NOISE[i]);
    }
    for(i = 0; i < 4; i++){
        DR_NOISE[i] = markov(DR_NOISE_sigma[i], DR_NOISE_bcor[i], get_rettime(), int_step, DR_NOISE[i]);
    }
}

void GPS_Receiver::get_quadriga(){
    int i(0);
    int j(0);
    int m(0);

    gdop = LARGE;
    int quad[4] = {0, 0, 0, 0};  // location of quadriga SVs
                                 // in 'ssii_vis[visible_count]

    // conversion to inertial (J2000) coordinates
    double ssii[24][4];
    double sin_incl = sin(gps_sats->inclination);
    double cos_incl = cos(gps_sats->inclination);
    for (i = 0; i < 24; i++) {
        ssii[i][0] =
            gps_sats->radius * (cos(gps_sats->sv_data[i][0]) * cos(gps_sats->sv_data[i][1]) -
                   sin(gps_sats->sv_data[i][0]) * sin(gps_sats->sv_data[i][1]) * cos_incl);
        ssii[i][1] =
            gps_sats->radius * (sin(gps_sats->sv_data[i][0]) * cos(gps_sats->sv_data[i][1]) +
                   cos(gps_sats->sv_data[i][0]) * sin(gps_sats->sv_data[i][1]) * cos_incl);
        ssii[i][2] = gps_sats->radius * sin(gps_sats->sv_data[i][1]) * sin_incl;
        ssii[i][3] = 0;
        // last entry is a flag with the code:
        // =0:not visible; >0:visible (not int but double!), where the
        // number
        // is the SV slot# (1,2,3,...,24)
    }

    // determining visible satellites
    arma::vec3 SSII;
    arma::vec3 SBII = newton->get_SBII();
    int visible_count(0);

    for (i = 0; i < 24; i++) {
        SSII[0] = ssii[i][0];
        SSII[1] = ssii[i][1];
        SSII[2] = ssii[i][2];

        // SV to user angle with vertex at Earth center
        double delta = angle(SSII, SBII);

        // grazing angle of SV beam with vertex at Earth center
        double epsilon = acos((Earth::radius + del_rearth) / gps_sats->radius);

        // min radius of user to have clear line-of-site to SV
        double rmin = (Earth::radius + del_rearth) / cos(delta - epsilon);

        if (delta < epsilon) {
            // SV is in the visibility cone
            ssii[i][3] = i + 1;
            visible_count++;
        } else {
            // user is outside the visibility cone but high enough to have
            // LOS to the SV
            // (rmin can go negative if delta-epsilon>90deg; this can happen
            // when the user is opposite (or nearly opposite) of the SV;
            // this is always a no-visibility case)
            double dbi = norm(SBII);
            if (rmin > 0 && rmin < dbi) {
                ssii[i][3] = i + 1;
                visible_count++;
            }
        }
    }
    // re-acquiring GPS if not enough SVs visible (less than 4)
    if (visible_count < 4) {
        gps_epoch = get_rettime();
        gps_acq = false;
    }
    // selecting best 4 SVs if 4 or more are visible
    else {

        // repackage visible SVs into 'ssii_vis' single-dimensioned array
        // inertial displacement vector elements are stored sequentially in
        // 'ssii_vis[4*visible_count]'
        // 'ssii_vis' has 3 inertial coordinates and SV slot# of all visible
        // SVs
        double *ssii_vis;
        ssii_vis = new double[4 * visible_count];
        int k(0);
        for (i = 0; i < 24; i++) {
            if (ssii[i][3] > 0) {
                *(ssii_vis + k) = ssii[i][0];
                *(ssii_vis + k + 1) = ssii[i][1];
                *(ssii_vis + k + 2) = ssii[i][2];
                *(ssii_vis + k + 3) = ssii[i][3];
                k = k + 4;
            }
        }
        // selecting quadriga (four SVs) with smallest GDOP
        // i1, i2, i3, i4 are the SVs picked by the binomial combination
        int nm3 = visible_count - 3;  // nm3=1
        int nm2 = visible_count - 2;  // nm2=2
        int nm1 = visible_count - 1;  // nm1=3

        for (int i1 = 0; i1 < nm3; i1++) {
            for (int i2 = i1 + 1; i2 < nm2; i2++) {
                for (int i3 = i2 + 1; i3 < nm1; i3++) {
                    for (int i4 = i3 + 1; i4 < visible_count; i4++) {
                        // pullling the quadriga inertial coordinates
                        arma::vec3 SSII1;
                        arma::vec3 SSII2;
                        arma::vec3 SSII3;
                        arma::vec3 SSII4;
                        for (m = 0; m < 3; m++) {
                            SSII1[m] = *(ssii_vis + 4 * i1 + m);
                            SSII2[m] = *(ssii_vis + 4 * i2 + m);
                            SSII3[m] = *(ssii_vis + 4 * i3 + m);
                            SSII4[m] = *(ssii_vis + 4 * i4 + m);
                        }
                        // calculating user wrt the SV displacement unit
                        // vectors
                        arma::vec UNI1 = normalise(SBII - SSII1);
                        arma::vec UNI2 = normalise(SBII - SSII2);
                        arma::vec UNI3 = normalise(SBII - SSII3);
                        arma::vec UNI4 = normalise(SBII - SSII4);

                        // building the GPS 'H' matrix
                        arma::mat44 HGPS(arma::fill::ones);
                        HGPS.col(0) = UNI1;
                        HGPS.col(1) = UNI2;
                        HGPS.col(2) = UNI3;
                        HGPS.col(3) = UNI4;
                        // calculating GDOP
                        arma::mat44 COV;
                        COV = arma::inv(HGPS * arma::trans(HGPS));
                        double gdop_local = sqrt(sum(COV.diag()));

                        // save slot # of quadriga SVs if GDOP has decreased
                        if (gdop_local < gdop) {
                            gdop = gdop_local;
                            quad[0] = i1;
                            quad[1] = i2;
                            quad[2] = i3;
                            quad[3] = i4;
                        }
                    }
                }
            }
        }  // end of picking quadriga amongst visible SVs

        // extracting "best" quadriga from visible SVs
        // and storing inertial coordinates of the four SVs and their slot#
        // in ssii_quad[16]
        for (m = 0; m < 4; m++) {
            for (int n = 0; n < 4; n++) {
                *(ssii_quad + 4 * m + n) = *(ssii_vis + 4 * quad[m] + n);
            }
        }
        delete[] ssii_vis;

        // calculating inertial velocity of quadriga SVs
        // getting slot# of quadriga
        //
        int islot[i];
        for (i = 0; i < 4; i++) {
            slot[i] = *(ssii_quad + 4 * i + 3);
            // casting into an int
            islot[i] = (int)slot[i];
        }
        // storing inertial velocities of the four SVs in vsii_quad[12]
        double sin_incl = sin(gps_sats->inclination);
        double cos_incl = cos(gps_sats->inclination);
        double vel = gps_sats->radius * gps_sats->angular_velocity;
        for (m = 0; m < 4; m++) {
            int ii = islot[m] - 1;  // reminder: slot#=1,2,3...,24
            *(vsii_quad + 3 * m + 0) =
                vel *
                (-sin(gps_sats->sv_data[ii][1]) * cos(gps_sats->sv_data[ii][0]) -
                 cos(gps_sats->sv_data[ii][1]) * sin(gps_sats->sv_data[ii][0]) * cos_incl);
            *(vsii_quad + 3 * m + 1) =
                vel *
                (-sin(gps_sats->sv_data[ii][1]) * sin(gps_sats->sv_data[ii][0]) +
                 cos(gps_sats->sv_data[ii][1]) * cos(gps_sats->sv_data[ii][0]) * cos_incl);
            *(vsii_quad + 3 * m + 2) =
                vel * (cos(gps_sats->sv_data[ii][1]) * sin_incl);
        }
    }  // end of picking quadriga from 4 or more visible SVs
}

void GPS_Receiver::filter_extrapolation(double int_step){
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

void GPS_Receiver::measure(){
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

    /* GPS Update and Measurement */
    double slotm(0);

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
        // unpacking i-th SV inertial position
        arma::vec3 SSII;
        for (int j = 0; j < 3; j++) {
            SSII[j] = *(ssii_quad + 4 * i + j);
        }
        // Z150126 - start
        // diagnostics: getting long, lat, alt of the four quadriga SVs for
        // plotting in GLOBE
        double lon(0), lat(0), alt(0);
        cad::geo84_in(lon, lat, alt, SSII, gps_sats->time);
        switch(i){
            case 0:
                lon1 = lon * DEG;
                lat1 = lat * DEG;
                alt1 = alt;
                break;
            case 1:
                lon2 = lon * DEG;
                lat2 = lat * DEG;
                alt2 = alt;
                break;
            case 2:
                lon3 = lon * DEG;
                lat3 = lat * DEG;
                alt3 = alt;
                break;
            case 3:
                lon4 = lon * DEG;
                lat4 = lat * DEG;
                alt4 = alt;
                break;
        }

        arma::vec3 SBII = newton->get_SBII();
        arma::vec3 VBII = newton->get_VBII();
        arma::vec3 WBII = euler->get_WBII();

        arma::vec3 SBIIC = grab_SBIIC();
        arma::vec3 VBIIC = grab_VBIIC();
        arma::vec3 WBICI = grab_WBICI();

        // calculating true range to SV
        arma::vec3 SSBI;

        SSBI = SSII - SBII;
        double dsb = norm(SSBI);

        // measured pseudo-range
        double dsb_meas = dsb + PR_BIAS[i] + PR_NOISE[i] + ucbias_error;

        // unpacking i-th SV inertial velocity
        arma::vec3 VSII;
        for (int j = 0; j < 3; j++) {
            VSII[j] = *(vsii_quad + 3 * i + j);
        }
        // velocity of SV wrt user
        arma::vec3 VSBI;
        VSBI = VSII - VBII - skew_sym(WBII) * SSBI;

        // calculating true range-rate to SV
        arma::vec3 USSBI;
        USSBI = SSBI * (1 / dsb);
        double dvsb = dot(VSBI, USSBI);

        // measured delta-range rate
        double dvsb_meas = dvsb + DR_NOISE[i] + ucfreq_error;

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

        // for diagnostics: loading the 4 SV slot # of the quadriga
        *(slot + i) = *(ssii_quad + 4 * i + 3);
        // accumulating sum of slots
        slotm = slotm + slot[i];
    }
    // for diagnostics displaying the SV slot# of the quadriga on the
    // console
    // but only if they have changed (i.e., sum of slot# has changed)

    if (slotsum != slotm) {
        slotsum = slotm;
        std::cout << " *** GPS Quadriga slot # " << slot[0] << "  "
                  << slot[1] << "  " << slot[2] << "  " << slot[3]
                  << " ;  GDOP = " << gdop << " m ***\n";
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
    KK = inv(PP * trans(HH) * (HH * PP * trans(HH) + RR));
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

arma::vec3 GPS_Receiver::get_SXH() { return SXH; };
arma::vec3 GPS_Receiver::get_VXH() { return VXH; };
arma::vec3 GPS_Receiver::get_CXH() { return CXH; };
