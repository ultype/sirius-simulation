#include "math_utility.hh"
#include "integrate.hh"

#include "Tvc.hh"
#include "sim_services/include/simtime.h"
#include "dsp_can_interfaces.h"

TVC::TVC()
    :   VECTOR_INIT(FPB, 3),
        VECTOR_INIT(FMPB, 3),
        MATRIX_INIT(TS2_N1_B, 3, 3),
        MATRIX_INIT(TS2_N2_B, 3, 3),
        MATRIX_INIT(TS2_N3_B, 3, 3),
        MATRIX_INIT(TS2_N4_B, 3, 3),
        MATRIX_INIT(TS3_N1_B, 3, 3),
        MATRIX_INIT(TS3_N2_B, 3, 3),
        VECTOR_INIT(r_N1, 3),
        VECTOR_INIT(r_N2, 3),
        VECTOR_INIT(r_N3, 3),
        VECTOR_INIT(r_N4, 3),
        VECTOR_INIT(Q_TVC, 6),
        VECTOR_INIT(S2_FPB1, 3),
        VECTOR_INIT(S2_FPB2, 3),
        VECTOR_INIT(S2_FPB3, 3),
        VECTOR_INIT(S2_FPB4, 3),
        VECTOR_INIT(S2_FMPB1, 3),
        VECTOR_INIT(S2_FMPB2, 3),
        VECTOR_INIT(S2_FMPB3, 3),
        VECTOR_INIT(S2_FMPB4, 3),
        VECTOR_INIT(lx, 3) {
    this->default_data();
    this->s2_act1_y1 = 0.0;
    this->s2_act2_y1 = 0.0;
    this->s2_act3_y1 = 0.0;
    this->s2_act4_y1 = 0.0;
    this->s3_act1_y1 = 0.0;
    this->s3_act2_y1 = 0.0;
    this->s3_act3_y1 = 0.0;
    this->s3_act4_y1 = 0.0;

    this->s2_act1_y1_saturation = 0.0;
    this->s2_act2_y1_saturation = 0.0;
    this->s2_act3_y1_saturation = 0.0;
    this->s2_act4_y1_saturation = 0.0;
    this->s3_act1_y1_saturation = 0.0;
    this->s3_act2_y1_saturation = 0.0;
    this->s3_act3_y1_saturation = 0.0;
    this->s3_act4_y1_saturation = 0.0;

    this->s2_act1_rate_old = 0.0;
    this->s2_act2_rate_old = 0.0;
    this->s2_act3_rate_old = 0.0;
    this->s2_act4_rate_old = 0.0;
    this->s3_act1_rate_old = 0.0;
    this->s3_act2_rate_old = 0.0;
    this->s3_act3_rate_old = 0.0;
    this->s3_act4_rate_old = 0.0;

    this->s2_act1_rate_saturation_old = 0.0;
    this->s2_act2_rate_saturation_old = 0.0;
    this->s2_act3_rate_saturation_old = 0.0;
    this->s2_act4_rate_saturation_old = 0.0;
    this->s3_act1_rate_saturation_old = 0.0;
    this->s3_act2_rate_saturation_old = 0.0;
    this->s3_act3_rate_saturation_old = 0.0;
    this->s3_act4_rate_saturation_old = 0.0;
}

TVC::TVC(const TVC& other)
    :   VECTOR_INIT(FPB, 3),
        VECTOR_INIT(FMPB, 3) {
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

TVC& TVC::operator=(const TVC& other) {
    if (&other == this)
        return *this;

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

void TVC::default_data() {
}

void TVC::initialize() {
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

arma::vec3 TVC::calculate_FPB(double eta, double zet, double thrust) {
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

arma::vec3 TVC::calculate_FMPB(double xcg) {
    arma::vec3 __FMPB;

    double arm = parm - xcg;
    __FMPB[0] = 0;
    __FMPB[1] = arm * FPB[2];
    __FMPB[2] = -arm * FPB[1];

    return __FMPB;
}

void TVC::decode_frame(struct can_frame *pframe) {
    struct tvc_param_t *buff = NULL;
    if (pframe == NULL)
        goto empty_frame;
    debug_hex_dump("Get from ring", reinterpret_cast<uint8_t *>(pframe), 16);
    switch (pframe->can_id) {
        case FC_to_TVC_II_NO1:
        case FC_to_TVC_III_NO1:
            buff = &tvc_no1;
            break;
        case FC_to_TVC_II_NO2:
        case FC_to_TVC_III_NO2:
            buff = &tvc_no2;
            break;
        default:
            fprintf(stderr, "[%d:%s] Unkonw TARGET_ID.\n", __LINE__, __FUNCTION__);
    }
    if (buff == NULL)
        goto empty_frame;
    switch (pframe->data[0]) {
        case TVC_MOVEMENT_FAKE:
        case TVC_MOVEMENT_REAL:
        /* TVC no1 pitch_count => theta_a */
        /* TVC no1 yaw_count => theta_b */
        /* TVC no2 pitch_count => theta_c */
        /* TVC no2 yaw_count => theta_d */
            copy_buffer_ntohs(reinterpret_cast<uint16_t *>(&buff->pitch_count), reinterpret_cast<uint8_t *>(&pframe->data[1]));
            copy_buffer_ntohs(reinterpret_cast<uint16_t *>(&buff->yaw_count), reinterpret_cast<uint8_t *>(&pframe->data[3]));
            break;
        default:
            fprintf(stderr, "[%d:%s] Unkonw TASK_ID.\n", __LINE__, __FUNCTION__);
    }
empty_frame:
        return;
}

void TVC::actuate(double int_step, struct icf_ctrlblk_t* C) {
    // local variables
    double eta(0), zet(0);
    double etac(0), zetc(0);

    // input from other modules
    double pdynmc = grab_pdynmc();
    arma::vec3 xcg    = grab_xcg();
    double thrust = grab_thrust();
    double delecx = 0.0;  // grab_delecx();
    double delrcx = 0.0;  // grab_delrcx();
    double alphax = grab_alphax();
#ifdef CONFIG_SIL_ENABLE
    theta_a_cmd = grab_theta_a_cmd();
    theta_b_cmd = grab_theta_b_cmd();
    theta_c_cmd = grab_theta_c_cmd();
    theta_d_cmd = grab_theta_d_cmd();
#else
    struct can_frame frame_tvc_no1;
    struct can_frame frame_tvc_no2;
    if (icf_rx_dequeue(C, EGSE_TVC_SW_QIDX, &frame_tvc_no1, sizeof(struct can_frame)) > 0) {
        decode_frame(&frame_tvc_no1);
        if (OUT_RANGE(tvc_no1.pitch_count, -TVC_ROTATION_LIMIT_CNT, TVC_ROTATION_LIMIT_CNT)) {
            fprintf(stderr, "tvc_no1.pitch_count: %d\n", tvc_no1.pitch_count);
            goto discard;
        }
        if (OUT_RANGE(tvc_no1.yaw_count, -TVC_ROTATION_LIMIT_CNT, TVC_ROTATION_LIMIT_CNT)) {
            fprintf(stderr, "tvc_no1.yaw_count: %d\n", tvc_no1.yaw_count);
            goto discard;
        }
    }
    if (icf_rx_dequeue(C, EGSE_TVC_SW_QIDX, &frame_tvc_no2, sizeof(struct can_frame)) > 0) {
        decode_frame(&frame_tvc_no2);
        if (OUT_RANGE(tvc_no2.pitch_count, -TVC_ROTATION_LIMIT_CNT, TVC_ROTATION_LIMIT_CNT)) {
            fprintf(stderr, "tvc_no2.pitch_count: %d\n", tvc_no2.pitch_count);
            goto discard;
        }
        if (OUT_RANGE(tvc_no2.yaw_count, -TVC_ROTATION_LIMIT_CNT, TVC_ROTATION_LIMIT_CNT)) {
            fprintf(stderr, "tvc_no2.yaw_count: %d\n", tvc_no2.yaw_count);
            goto discard;
        }
    }

    //  fprintf(stderr, "TVC::actuate %d %d %d %d\n", tvc_no1.pitch_count,tvc_no1.yaw_count, tvc_no2.pitch_count, tvc_no2.yaw_count);
    //  fprintf(stderr, "TVC::actuate %f %f %f %f\n", theta_a_cmd, theta_b_cmd, theta_c_cmd, theta_d_cmd);
    theta_a_cmd = tvc_no1.pitch_count * TVC_DSP_RESOLUTION * (PI/180);
    theta_b_cmd = tvc_no1.yaw_count * TVC_DSP_RESOLUTION * (PI/180);
    theta_c_cmd = tvc_no2.pitch_count * TVC_DSP_RESOLUTION * (PI/180);
    theta_d_cmd = tvc_no2.yaw_count * TVC_DSP_RESOLUTION * (PI/180);
discard:
#endif  // CONFIG_SIL_ENABLE

    switch (mtvc) {
        case NO_TVC:
            // return if no tvc
            // thrust forces in body axes
            return;
            break;
        case NO_DYNAMIC_TVC:
            eta = delecx * RAD;
            zet = delrcx * RAD;
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
        case S2_TVC:
            S2_actuator_1(theta_a_cmd, int_step);
            S2_actuator_2(theta_b_cmd, int_step);
            S2_actuator_3(theta_c_cmd, int_step);
            S2_actuator_4(theta_d_cmd, int_step);

            calculate_S2_Q(s2_act1_y2_saturation, s2_act2_y2_saturation, s2_act3_y2_saturation, s2_act4_y2_saturation);

            // this->FPB = calculate_S2_FPB(s2_act1_y2_saturation, s2_act2_y2_saturation, s2_act3_y2_saturation, s2_act4_y2_saturation, thrust);
             // thrust moments in body axes
            // this->FMPB = calculate_S2_FMPB(s2_act1_y2_saturation, s2_act2_y2_saturation, s2_act3_y2_saturation, s2_act4_y2_saturation, thrust, xcg);
            return;
            break;
        case S3_TVC:
            S3_actuator_1(theta_a_cmd, int_step);
            S3_actuator_2(theta_b_cmd, int_step);
            // S3_actuator_3(theta_c_cmd, int_step);
            // S3_actuator_4(theta_d_cmd, int_step);

            calculate_S3_Q(s3_act1_y2_saturation, s3_act2_y2_saturation);  // a = yaw, b = pitch
            // this->FPB = calculate_S3_FPB(s3_act1_y2_saturation, s3_act2_y2_saturation, s3_act3_y2_saturation, s3_act4_y2_saturation, thrust);
            //  // thrust moments in body axes
            // this->FMPB = calculate_S3_FMPB(s3_act1_y2_saturation, s3_act2_y2_saturation, s3_act3_y2_saturation, s3_act4_y2_saturation, thrust, xcg);
            return;
            break;
        default:
            break;
    }

    // thrust forces in body axes
    // this->FPB = calculate_FPB(eta, zet, thrust);

    // // thrust moments in body axes
    // this->FMPB = calculate_FMPB(xcg);

    // // output
    // etax = eta * DEG;
    // zetx = zet * DEG;

    // // diagnostic
    // etacx = etac * DEG;
    // zetcx = zetc * DEG;
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

std::tuple<double, double> TVC::tvc_scnd(double etac, double zetc, double int_step) {
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
void TVC::set_parm(double in) { parm = in; }
void TVC::set_s2_tvc_acc_lim(double in) { s2_acclim = in;}
void TVC::set_s3_tvc_acc_lim(double in) { s3_acclim = in;}
void TVC::set_s2_tvc_d(double in) { s2_d = in; }
void TVC::set_s3_tvc_d(double in) { s3_d = in; }
void TVC::set_S2_reference_p(double in) { s2_reference_p = in; }
void TVC::set_S3_reference_p(double in) { s3_reference_p = in; }

double TVC::get_parm() { return parm; }
arma::vec3 TVC::get_lx() { return lx; }

arma::vec3 TVC::get_FPB() { return FPB; }
arma::vec3 TVC::get_FMPB() { return FMPB; }
arma::vec6 TVC::get_Q_TVC() { return Q_TVC; }

void TVC::calculate_S2_Q(double theta_a, double theta_b, double theta_c, double theta_d) {
    arma::mat33 TBI = grab_TBI();
    arma::vec3 xcg = grab_xcg();
    double thrust = grab_thrust();
    arma::vec3 F1, F2, F3, F4;
    arma::vec3 ga_1, ga_2, ga_3;
    arma::vec3 be_4, be_5, be_6;

    lx(0) = xcg(0) - (s2_reference_p);
    lx(1) = xcg(1);
    lx(2) = xcg(2);

    ga_1(0) = be_4(0) = 1.0;
    ga_1(1) = be_4(1) = 0.0;
    ga_1(2) = be_4(2) = 0.0;

    ga_2(0) = be_5(0) = 0.0;
    ga_2(1) = be_5(1) = 1.0;
    ga_2(2) = be_5(2) = 0.0;

    ga_3(0) = be_6(0) = 0.0;
    ga_3(1) = be_6(1) = 0.0;
    ga_3(2) = be_6(2) = 1.0;

    F1(0) = F2(0) = F3(0) = F4(0) = thrust / 4.;
    F1(1) = F2(1) = F3(1) = F4(1) = 0.0;
    F1(2) = F2(2) = F3(2) = F4(2) = 0.0;

    TS2_N1_B(0, 0) = cos(theta_a);
    TS2_N1_B(0, 1) = sin(theta_a);
    TS2_N1_B(0, 2) = 0.0;
    TS2_N1_B(1, 0) = -sin(theta_a);
    TS2_N1_B(1, 1) = cos(theta_a);
    TS2_N1_B(1, 2) = 0.0;
    TS2_N1_B(2, 0) = 0.0;
    TS2_N1_B(2, 1) = 0.0;
    TS2_N1_B(2, 2) = 1.0;

    TS2_N2_B(0, 0) = cos(theta_b);
    TS2_N2_B(0, 1) = 0.0;
    TS2_N2_B(0, 2) = -sin(theta_b);
    TS2_N2_B(1, 0) = 0.0;
    TS2_N2_B(1, 1) = 1.0;
    TS2_N2_B(1, 2) = 0.0;
    TS2_N2_B(2, 0) = sin(theta_b);
    TS2_N2_B(2, 1) = 0.0;
    TS2_N2_B(2, 2) = cos(theta_b);

    TS2_N3_B(0, 0) = cos(theta_c);
    TS2_N3_B(0, 1) = sin(theta_c);
    TS2_N3_B(0, 2) = 0.0;
    TS2_N3_B(1, 0) = -sin(theta_c);
    TS2_N3_B(1, 1) = cos(theta_c);
    TS2_N3_B(1, 2) = 0.0;
    TS2_N3_B(2, 0) = 0.0;
    TS2_N3_B(2, 1) = 0.0;
    TS2_N3_B(2, 2) = 1.0;

    TS2_N4_B(0, 0) = cos(theta_d);
    TS2_N4_B(0, 1) = 0.0;
    TS2_N4_B(0, 2) = -sin(theta_d);
    TS2_N4_B(1, 0) = 0.0;
    TS2_N4_B(1, 1) = 1.0;
    TS2_N4_B(1, 2) = 0.0;
    TS2_N4_B(2, 0) = sin(theta_d);
    TS2_N4_B(2, 1) = 0.0;
    TS2_N4_B(2, 2) = cos(theta_d);

    r_N1(0) = 0.0;
    r_N1(1) = 0.0;
    r_N1(2) = -s2_d;

    r_N2(0) = 0.0;
    r_N2(1) = s2_d;
    r_N2(2) = 0.0;

    r_N3(0) = 0.0;
    r_N3(1) = 0.0;
    r_N3(2) = s2_d;

    r_N4(0) = 0.0;
    r_N4(1) = -s2_d;
    r_N4(2) = 0.0;

    Q_TVC(0) = dot((trans(TBI) * trans(TS2_N1_B) * F1 + trans(TBI) * trans(TS2_N2_B) * F2
                + trans(TBI) * trans(TS2_N3_B) * F3 + trans(TBI) * trans(TS2_N4_B) * F4),  ga_1);
    Q_TVC(1) = dot((trans(TBI) * trans(TS2_N1_B) * F1 + trans(TBI) * trans(TS2_N2_B) * F2
                + trans(TBI) * trans(TS2_N3_B) * F3 + trans(TBI) * trans(TS2_N4_B) * F4), ga_2);
    Q_TVC(2) = dot((trans(TBI) * trans(TS2_N1_B) * F1 + trans(TBI) * trans(TS2_N2_B) * F2
                + trans(TBI) * trans(TS2_N3_B) * F3 + trans(TBI) * trans(TS2_N4_B) * F4), ga_3);
    Q_TVC(3) = dot(trans(TBI) * trans(TS2_N1_B) * F1, -trans(TBI) * cross_matrix(r_N1) * be_4) + dot(trans(TBI) * trans(TS2_N2_B) * F2, -trans(TBI) * cross_matrix(r_N2) * be_4)
                + dot(trans(TBI) * trans(TS2_N3_B) * F3, -trans(TBI) * cross_matrix(r_N3) * be_4) + dot(trans(TBI) * trans(TS2_N4_B) * F4, -trans(TBI) * cross_matrix(r_N4) * be_4);
    Q_TVC(4) = dot(trans(TBI) * trans(TS2_N1_B) * F1, -trans(TBI) * cross_matrix(r_N1) * be_5) + dot(trans(TBI) * trans(TS2_N2_B) * F2, -trans(TBI) * cross_matrix(r_N2) * be_5)
                + dot(trans(TBI) * trans(TS2_N3_B) * F3, -trans(TBI) * cross_matrix(r_N3) * be_5) + dot(trans(TBI) * trans(TS2_N4_B) * F4, -trans(TBI) * cross_matrix(r_N4) * be_5);
    Q_TVC(5) = dot(trans(TBI) * trans(TS2_N1_B) * F1, -trans(TBI) * cross_matrix(r_N1) * be_6) + dot(trans(TBI) * trans(TS2_N2_B) * F2, -trans(TBI) * cross_matrix(r_N2) * be_6)
                + dot(trans(TBI) * trans(TS2_N3_B) * F3, -trans(TBI) * cross_matrix(r_N3) * be_6) + dot(trans(TBI) * trans(TS2_N4_B) * F4, -trans(TBI) * cross_matrix(r_N4) * be_6);
}

void TVC::calculate_S3_Q(double theta_a, double theta_b) {
    arma::mat33 TBI = grab_TBI();
    arma::vec3 xcg = grab_xcg();
    double thrust = grab_thrust();
    arma::vec3 F1, F2;
    arma::vec3 ga_1, ga_2, ga_3;
    arma::vec3 be_4, be_5, be_6;

    lx(0) = xcg(0) - (s3_reference_p);
    lx(1) = xcg(1);
    lx(2) = xcg(2);

    ga_1(0) = be_4(0) = 1.0;
    ga_1(1) = be_4(1) = 0.0;
    ga_1(2) = be_4(2) = 0.0;

    ga_2(0) = be_5(0) = 0.0;
    ga_2(1) = be_5(1) = 1.0;
    ga_2(2) = be_5(2) = 0.0;

    ga_3(0) = be_6(0) = 0.0;
    ga_3(1) = be_6(1) = 0.0;
    ga_3(2) = be_6(2) = 1.0;

    F1(0) = thrust;
    F1(1) = 0.0;
    F1(2) = 0.0;

    TS3_N1_B(0, 0) = cos(theta_b) * cos(theta_a);
    TS3_N1_B(0, 1) = cos(theta_b) * sin(theta_a);
    TS3_N1_B(0, 2) = -sin(theta_b);
    TS3_N1_B(1, 0) = -sin(theta_a);
    TS3_N1_B(1, 1) = cos(theta_a);
    TS3_N1_B(1, 2) = 0.0;
    TS3_N1_B(2, 0) = sin(theta_b) * cos(theta_a);
    TS3_N1_B(2, 1) = sin(theta_b) * sin(theta_a);
    TS3_N1_B(2, 2) = cos(theta_b);

    // TS3_N2_B(0, 0) = cos(theta_c) * cos(theta_d);
    // TS3_N2_B(0, 1) = cos(theta_c) * sin(theta_d);
    // TS3_N2_B(0, 2) = -sin(theta_c);
    // TS3_N2_B(1, 0) = -sin(theta_d);
    // TS3_N2_B(1, 1) = cos(theta_d);
    // TS3_N2_B(1, 2) = 0.0;
    // TS3_N2_B(2, 0) = sin(theta_c) * cos(theta_d);
    // TS3_N2_B(2, 1) = sin(theta_c) * sin(theta_d);
    // TS3_N2_B(2, 2) = cos(theta_c);

    // r_N1(0) = 0.0;
    // r_N1(1) = 0.0;
    // r_N1(2) = -s3_d;

    // r_N2(0) = 0.0;
    // r_N2(1) = 0.0;
    // r_N2(2) = s3_d;

    Q_TVC(0) = dot((trans(TBI) * trans(TS3_N1_B)) * F1, ga_1);
    Q_TVC(1) = dot((trans(TBI) * trans(TS3_N1_B)) * F1, ga_2);
    Q_TVC(2) = dot((trans(TBI) * trans(TS3_N1_B)) * F1, ga_3);
    Q_TVC(3) = 0.0;
    Q_TVC(4) = 0.0;
    Q_TVC(5) = 0.0;
}

// arma::vec3 TVC::calculate_S2_FPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust) {
//     arma::vec3 S2_FPB;

//     // S2_FPB(0) = (cos(theta_a) + cos(theta_b) + cos(theta_c) + cos(theta_d)) * (thrust / 4.0);
//     // S2_FPB(1) = (sin(theta_a) + sin(theta_c)) * (thrust / 4.0);
//     // S2_FPB(2) = (-sin(theta_b) - sin(theta_d)) * (thrust / 4.0);
//     S2_FPB1(0) = cos(theta_a) * (thrust / 4.0);
//     S2_FPB1(1) = sin(theta_a) * (thrust / 4.0);
//     S2_FPB1(2) = 0.0;

//     S2_FPB2(0) = cos(theta_b) * (thrust / 4.0);
//     S2_FPB2(1) = 0.0;
//     S2_FPB2(2) = -sin(theta_b) * (thrust / 4.0);

//     S2_FPB3(0) = cos(theta_c) * (thrust / 4.0);
//     S2_FPB3(1) = sin(theta_c) * (thrust / 4.0);
//     S2_FPB3(2) = 0.0;

//     S2_FPB4(0) = cos(theta_d) * (thrust / 4.0);
//     S2_FPB4(1) = 0.0;
//     S2_FPB4(2) = -sin(theta_d) * (thrust / 4.0);

//     S2_FPB = S2_FPB1 + S2_FPB2 + S2_FPB3 + S2_FPB4;


//     return S2_FPB;
// }

// arma::vec3 TVC::calculate_S2_FMPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust, double xcg) {
//     arma::vec3 S2_FMPB;
//     double d(0.69);

//     lx = -xcg - (-8.436);  // xcg minus because the SE's coordinate definition

//     // S2_FMPB(0) = ((-0.5 * d * sin(theta_b) + 0.5 * d * sin(theta_d)) - (-0.5 * d * sin(theta_a) + 0.5 * d * sin(theta_c))) * (thrust / 4.0);
//     // S2_FMPB(1) = ((-lx * sin(theta_b) - lx * sin(theta_d))) * (thrust / 4.0);
//     // S2_FMPB(2) = ((-lx * sin(theta_a) - lx * sin(theta_c))) * (thrust / 4.0);
//     S2_FMPB(0) = S2_FPB2(2) * 0.5 * d + (-S2_FPB4(2) * 0.5 * d) - (-S2_FPB1(1) * 0.5 * d) - (S2_FPB3(1) * 0.5 * d);
//     S2_FMPB(1) = -0.5 * d * S2_FPB1(0) + 0.5 * d * S2_FPB3(0) - (-lx * S2_FPB2(2)) - (-lx * S2_FPB4(2));
//     S2_FMPB(2) = -lx * S2_FPB1(1) + (-lx * S2_FPB3(1)) - (0.5 * d * S2_FPB2(0)) - (-0.5 * d * S2_FPB4(0));

//     return S2_FMPB;
// }

// arma::vec3 TVC::calculate_S3_FPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust) {
//     arma::vec3 S3_FPB;

//     S3_FPB(0) = (cos(theta_a) * cos(theta_b) + cos(theta_c) * cos(theta_d)) * (thrust / 2.0);
//     S3_FPB(1) = (sin(theta_a) * cos(theta_b) + cos(theta_c) * sin(theta_d)) * (thrust / 2.0);
//     S3_FPB(2) = (-sin(theta_b) - sin(theta_c)) * (thrust / 2.0);

//     return S3_FPB;
// }

// arma::vec3 TVC::calculate_S3_FMPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust, double xcg) {
//     arma::vec3 S3_FMPB;
//     double lx;
//     double d(0.4);

//     lx = -xcg - (-3.275);  // xcg minus because the SE's coordinate definition

//     S3_FMPB(0) = (0.5 * d * cos(theta_b) * sin(theta_a) - 0.5 * d * cos(theta_c) * sin(theta_d)) * (thrust / 2.0);
//     S3_FMPB(1) = (-0.5 * d * cos(theta_b) * cos(theta_a) + 0.5 * d * cos(theta_c) * cos(theta_d) - lx * sin(theta_b) - lx * sin(theta_c)) * (thrust / 2.0);
//     S3_FMPB(2) = (-lx * cos(theta_b) * sin(theta_a) - lx * cos(theta_c) * sin(theta_d)) * (thrust / 2.0);

//     return S3_FMPB;
// }

void TVC::set_S2_TVC() {
    this->mtvc = S2_TVC;
}

void TVC::set_S3_TVC() {
    this->mtvc = S3_TVC;
}

void TVC::S2_actuator_1(double command, double int_step) {
    s2_act1_rate = (s2_tau1 * command - s2_tau1 * s2_act1_y1);
    s2_act1_acc = (s2_act1_rate - s2_act1_rate_old) / int_step;

    if (fabs(s2_act1_acc) > s2_acclim) {
        if (s2_act1_acc > 0) {
            s2_act1_acc = s2_acclim;
        } else {
            s2_act1_acc = -s2_acclim;
        }
        s2_act1_rate = s2_act1_rate_old + s2_act1_acc * int_step;
    }

    if (fabs(s2_act1_rate) > s2_ratelim) {
        if (s2_act1_rate > 0) {
            s2_act1_rate = s2_ratelim;
        } else {
            s2_act1_rate = -s2_ratelim;
        }
    }
    // s2_act1_y2 = integrate(s2_act1_rate, s2_act1_rate_old, s2_act1_y1, int_step);
    s2_act1_y2 = s2_act1_y1 + s2_act1_rate * int_step;

    s2_act1_y1 = s2_act1_y2;
    s2_act1_rate_old = s2_act1_rate;

    s2_act1_y2_saturation = s2_act1_y2;
    if (fabs(s2_act1_y2_saturation) > s2_tvclim) {
        if (s2_act1_y2_saturation > 0) {
            s2_act1_y2_saturation = s2_tvclim;
        } else {
            s2_act1_y2_saturation = -s2_tvclim;
        }
    }

    s2_act1_rate_old = s2_act1_rate;
}

void TVC::S2_actuator_2(double command, double int_step) {
    s2_act2_rate = (s2_tau2 * command - s2_tau2 * s2_act2_y1);
    s2_act2_acc = (s2_act2_rate - s2_act2_rate_old) / int_step;

    if (fabs(s2_act2_acc) > s2_acclim) {
        if (s2_act2_acc > 0) {
            s2_act2_acc = s2_acclim;
        } else {
            s2_act2_acc = -s2_acclim;
        }
        s2_act2_rate = s2_act2_rate_old + s2_act2_acc * int_step;
    }

    if (fabs(s2_act2_rate) > s2_ratelim) {
        if (s2_act2_rate > 0) {
            s2_act2_rate = s2_ratelim;
        } else {
            s2_act2_rate = -s2_ratelim;
        }
    }
    // s2_act2_y2 = integrate(s2_act2_rate, s2_act2_rate_old, s2_act2_y1, int_step);
    s2_act2_y2 = s2_act2_y1 + s2_act2_rate * int_step;

    s2_act2_y1 = s2_act2_y2;
    s2_act2_rate_old = s2_act2_rate;

    s2_act2_y2_saturation = s2_act2_y2;
    if (fabs(s2_act2_y2_saturation) > s2_tvclim) {
        if (s2_act2_y2_saturation > 0) {
            s2_act2_y2_saturation = s2_tvclim;
        } else {
            s2_act2_y2_saturation = -s2_tvclim;
        }
    }

    s2_act2_rate_old = s2_act2_rate;
}

void TVC::S2_actuator_3(double command, double int_step) {
    s2_act3_rate = (s2_tau3 * command - s2_tau3 * s2_act3_y1);
    s2_act3_acc = (s2_act3_rate - s2_act3_rate_old) / int_step;

    if (fabs(s2_act3_acc) > s2_acclim) {
        if (s2_act3_acc > 0) {
            s2_act3_acc = s2_acclim;
        } else {
            s2_act3_acc = -s2_acclim;
        }
        s2_act3_rate = s2_act3_rate_old + s2_act3_acc * int_step;
    }

    if (fabs(s2_act3_rate) > s2_ratelim) {
        if (s2_act3_rate > 0) {
            s2_act3_rate = s2_ratelim;
        } else {
            s2_act3_rate = -s2_ratelim;
        }
    }
    // s2_act3_y2 = integrate(s2_act3_rate, s2_act3_rate_old, s2_act3_y1, int_step);
    s2_act3_y2 = s2_act3_y1 + s2_act3_rate * int_step;

    s2_act3_y1 = s2_act3_y2;
    s2_act3_rate_old = s2_act3_rate;

    s2_act3_y2_saturation = s2_act3_y2;
    if (fabs(s2_act3_y2_saturation) > s2_tvclim) {
        if (s2_act3_y2_saturation > 0) {
            s2_act3_y2_saturation = s2_tvclim;
        } else {
            s2_act3_y2_saturation = -s2_tvclim;
        }
    }

    s2_act3_rate_old = s2_act3_rate;
}

void TVC::S2_actuator_4(double command, double int_step) {
    s2_act4_rate = (s2_tau4 * command - s2_tau4 * s2_act4_y1);
    s2_act4_acc = (s2_act4_rate - s2_act4_rate_old) / int_step;

    if (fabs(s2_act4_acc) > s2_acclim) {
        if (s2_act4_acc > 0) {
            s2_act4_acc = s2_acclim;
        } else {
            s2_act4_acc = -s2_acclim;
        }
        s2_act4_rate = s2_act4_rate_old + s2_act4_acc * int_step;
    }

    if (fabs(s2_act4_rate) > s2_ratelim) {
        if (s2_act4_rate > 0) {
            s2_act4_rate = s2_ratelim;
        } else {
            s2_act4_rate = -s2_ratelim;
        }
    }
    // s2_act4_y2 = integrate(s2_act4_rate, s2_act4_rate_old, s2_act4_y1, int_step);
    s2_act4_y2 = s2_act4_y1 + s2_act4_rate * int_step;

    s2_act4_y1 = s2_act4_y2;
    s2_act4_rate_old = s2_act4_rate;

    s2_act4_y2_saturation = s2_act4_y2;
    if (fabs(s2_act4_y2_saturation) > s2_tvclim) {
        if (s2_act4_y2_saturation > 0) {
            s2_act4_y2_saturation = s2_tvclim;
        } else {
            s2_act4_y2_saturation = -s2_tvclim;
        }
    }

    s2_act4_rate_old = s2_act4_rate;
}

void TVC::S3_actuator_1(double command, double int_step) {
    s3_act1_rate = (s3_tau1 * command - s3_tau1 * s3_act1_y1);
    s3_act1_acc = (s3_act1_rate - s3_act1_rate_old) / int_step;

    if (fabs(s3_act1_acc) > s3_acclim) {
        if (s3_act1_acc > 0) {
            s3_act1_acc = s3_acclim;
        } else {
            s3_act1_acc = -s3_acclim;
        }
        s3_act1_rate = s3_act1_rate_old + s3_act1_acc * int_step;
    }

    if (fabs(s3_act1_rate) > s3_ratelim) {
        if (s3_act1_rate > 0) {
            s3_act1_rate = s3_ratelim;
        } else {
            s3_act1_rate = -s3_ratelim;
        }
    }
    s3_act1_y2 = integrate(s3_act1_rate, s3_act1_rate_old, s3_act1_y1, int_step);

    s3_act1_y1 = s3_act1_y2;
    s3_act1_rate_old = s3_act1_rate;

    s3_act1_y2_saturation = s3_act1_y2;
    if (fabs(s3_act1_y2_saturation) > s3_tvclim) {
        if (s3_act1_y2_saturation > 0) {
            s3_act1_y2_saturation = s3_tvclim;
        } else {
            s3_act1_y2_saturation = -s3_tvclim;
        }
    }
}

void TVC::S3_actuator_2(double command, double int_step) {
    s3_act2_rate = (s3_tau2 * command - s3_tau2 * s3_act2_y1);
    s3_act2_acc = (s3_act2_rate - s3_act2_rate_old) / int_step;

    if (fabs(s3_act2_acc) > s3_acclim) {
        if (s3_act2_acc > 0) {
            s3_act2_acc = s3_acclim;
        } else {
            s3_act2_acc = -s3_acclim;
        }
        s3_act2_rate = s3_act2_rate_old + s3_act2_acc * int_step;
    }

    if (fabs(s3_act2_rate) > s3_ratelim) {
        if (s3_act2_rate > 0) {
            s3_act2_rate = s3_ratelim;
        } else {
            s3_act2_rate = -s3_ratelim;
        }
    }
    s3_act2_y2 = integrate(s3_act2_rate, s3_act2_rate_old, s3_act2_y1, int_step);

    s3_act2_y1 = s3_act2_y2;
    s3_act2_rate_old = s3_act2_rate;

    s3_act2_y2_saturation = s3_act2_y2;
    if (fabs(s3_act2_y2_saturation) > s3_tvclim) {
        if (s3_act2_y2_saturation > 0) {
            s3_act2_y2_saturation = s3_tvclim;
        } else {
            s3_act2_y2_saturation = -s3_tvclim;
        }
    }
}

void TVC::S3_actuator_3(double command, double int_step) {
    s3_act3_rate = (s3_tau3 * command - s3_tau3 * s3_act3_y1);
    s3_act3_acc = (s3_act3_rate - s3_act3_rate_old) / int_step;

    if (fabs(s3_act3_acc) > s3_acclim) {
        if (s3_act3_acc > 0) {
            s3_act3_acc = s3_acclim;
        } else {
            s3_act3_acc = -s3_acclim;
        }
        s3_act3_rate = s3_act3_rate_old + s3_act3_acc * int_step;
    }

    if (fabs(s3_act3_rate) > s3_ratelim) {
        if (s3_act3_rate > 0) {
            s3_act3_rate = s3_ratelim;
        } else {
            s3_act3_rate = -s3_ratelim;
        }
    }
    s3_act3_y2 = integrate(s3_act3_rate, s3_act3_rate_old, s3_act3_y1, int_step);

    s3_act3_y1 = s3_act3_y2;
    s3_act3_rate_old = s3_act3_rate;

    s3_act3_y2_saturation = s3_act3_y2;
    if (fabs(s3_act3_y2_saturation) > s3_tvclim) {
        if (s3_act3_y2_saturation > 0) {
            s3_act3_y2_saturation = s3_tvclim;
        } else {
            s3_act3_y2_saturation = -s3_tvclim;
        }
    }
}

void TVC::S3_actuator_4(double command, double int_step) {
    s3_act4_rate = (s3_tau4 * command - s3_tau4 * s3_act4_y1);
    s3_act4_acc = (s3_act4_rate - s3_act4_rate_old) / int_step;

    if (fabs(s3_act4_acc) > s3_acclim) {
        if (s3_act4_acc > 0) {
            s3_act4_acc = s3_acclim;
        } else {
            s3_act4_acc = -s3_acclim;
        }
        s3_act4_rate = s3_act4_rate_old + s3_act4_acc * int_step;
    }

    if (fabs(s3_act4_rate) > s3_ratelim) {
        if (s3_act4_rate > 0) {
            s3_act4_rate = s3_ratelim;
        } else {
            s3_act4_rate = -s3_ratelim;
        }
    }
    s3_act4_y2 = integrate(s3_act4_rate, s3_act4_rate_old, s3_act4_y1, int_step);

    s3_act4_y1 = s3_act4_y2;
    s3_act4_rate_old = s3_act4_rate;

    s3_act4_y2_saturation = s3_act4_y2;
    if (fabs(s3_act4_y2_saturation) > s3_tvclim) {
        if (s3_act4_y2_saturation > 0) {
            s3_act4_y2_saturation = s3_tvclim;
        } else {
            s3_act4_y2_saturation = -s3_tvclim;
        }
    }
}

void TVC::set_s2_tau1(double in) { this->s2_tau1 = in; }
void TVC::set_s2_tau2(double in) { this->s2_tau2 = in; }
void TVC::set_s2_tau3(double in) { this->s2_tau3 = in; }
void TVC::set_s2_tau4(double in) { this->s2_tau4 = in; }
void TVC::set_s2_ratelim(double in) { this->s2_ratelim = in; }
void TVC::set_s2_tvclim(double in) { this->s2_tvclim = in; }
void TVC::set_s3_tau1(double in) { this->s3_tau1 = in; }
void TVC::set_s3_tau2(double in) { this->s3_tau2 = in; }
void TVC::set_s3_tau3(double in) { this->s3_tau3 = in; }
void TVC::set_s3_tau4(double in) { this->s3_tau4 = in; }
void TVC::set_s3_ratelim(double in) { this->s3_ratelim = in; }
void TVC::set_s3_tvclim(double in) { this->s3_tvclim = in; }

arma::mat33 TVC::cross_matrix(arma::vec3 in) {
    arma::mat33 c_matrix;

    c_matrix(0, 0) = 0.0;
    c_matrix(0, 1) = -in(2);
    c_matrix(0, 2) = in(1);
    c_matrix(1, 0) = in(2);
    c_matrix(1, 1) = 0.0;
    c_matrix(1, 2) = -in(0);
    c_matrix(2, 0) = -in(1);
    c_matrix(2, 1) = in(0);
    c_matrix(2, 2) = 0.0;

    return c_matrix;
}

double TVC::get_s2_act1_rate() { return s2_act1_rate; }
double TVC::get_s2_act2_rate() { return s2_act2_rate; }
double TVC::get_s2_act3_rate() { return s2_act3_rate; }
double TVC::get_s2_act4_rate() { return s2_act4_rate; }

double TVC::get_s2_act1_y2_saturation() { return s2_act1_y2_saturation; }
double TVC::get_s2_act2_y2_saturation() { return s2_act2_y2_saturation; }
double TVC::get_s2_act3_y2_saturation() { return s2_act3_y2_saturation; }
double TVC::get_s2_act4_y2_saturation() { return s2_act4_y2_saturation; }

double TVC::get_s2_act1_acc() { return s2_act1_acc; }
double TVC::get_s2_act2_acc() { return s2_act2_acc; }
double TVC::get_s2_act3_acc() { return s2_act3_acc; }
double TVC::get_s2_act4_acc() { return s2_act4_acc; }
