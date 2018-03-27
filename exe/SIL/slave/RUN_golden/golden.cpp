#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"
#include "../../../public/Modified_data/realtime.h"
#include "../Modified_data/gps_fc.h"
#include  "../../../models/gnc/include/DM_FSW_Interface.hh"
/* Stage2 Control Variable Constant */
const double S2_MDOT = 29.587;
const double S2_FMASS0 = 2958.7;
const double S2_XCG_1 = 4.7888;
const double S2_XCG_0 = 6.4138;
const double S2_ISP = 291.6145604;
const double S2_MOI_ROLL_0 = 517.8;
const double S2_MOI_ROLL_1 = 180.9;
const double S2_MOI_PITCH_0 = 32525.4;
const double S2_MOI_PITCH_1 = 19377.7;
const double S2_MOI_YAW_0 = 32519.8;
const double S2_MOI_YAW_1 = 19372.3;
const double S2_KPP = 3.0;
const double S2_KPI = 0.08;
const double S2_KPD = 0.01;
const double S2_KPPP = 6.25;
const double S2_PN = 1000.0;
const double S2_KRP = 3.0;
const double S2_KRI = 0.33;
const double S2_KRD = 0.015;
const double S2_KRPP = 9.375;
const double S2_RN = 1000.0;
const double S2_KYP = 3.0;
const double S2_KYI = 0.08;
const double S2_KYD = 0.01;
const double S2_KYPP = 7.5;
const double S2_YN = 1000.0;
const double S2_KAOAP = 3.0;
const double S2_KAOAI = 0.15;
const double S2_KAOAD = 0.01;
const double S2_KAOAPP = 9.375;
const double S2_AOAN = 1000.0;
const double S2_ROLLCMD = 0.0;
const double S2_PITCHCMD = -3.0;
const double S2_YAWCMD = 0.0;
const double S2_AOACMD = 0.0;

/* Stage3 Control Variable Constant*/
const double S3_MDOT = 3.814;
const double S3_FMASS0 = 381.4;
const double S3_XCG_1 = 2.5371;
const double S3_XCG_0 = 2.5808;
const double S3_ISP = 290;
const double S3_MOI_ROLL_0 = 65.3;
const double S3_MOI_ROLL_1 = 50.8;
const double S3_MOI_PITCH_0 = 633.6;
const double S3_MOI_PITCH_1 = 421.8;
const double S3_MOI_YAW_0 = 628.0;
const double S3_MOI_YAW_1 = 419.0;
const double S3_KPP = 3.0;
const double S3_KPI = 0.08;
const double S3_KPD = 0.01;
const double S3_KPPP = 6.25;
const double S3_PN = 1000.0;
const double S3_KRP = 3.0;
const double S3_KRI = 0.33;
const double S3_KRD = 0.015;
const double S3_KRPP = 9.375;
const double S3_RN = 1000.0;
const double S3_KYP = 3.0;
const double S3_KYI = 0.08;
const double S3_KYD = 0.01;
const double S3_KYPP = 7.5;
const double S3_YN = 1000.0;
const double S3_KAOAP = 3.0;
const double S3_KAOAI = 0.15;
const double S3_KAOAD = 0.01;
const double S3_KAOAPP = 9.375;
const double S3_AOAN = 1000.0;
const double S3_ROLLCMD = 0.0;
const double S3_PITCHCMD = 0.0;
const double S3_YAWCMD = 0.0;
const double S3_AOACMD = -0.0;


extern "C" int event_liftoff(void) {
    fc.ins.set_liftoff(1);
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_LIFTOFF;
    return 0;
}

extern "C" int event_control_rcs_on(void) {
    fc.control.set_S2_ROLL_CONTROL();
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_CONTROL_RCS_ON;
    return 0;
}

extern "C" int event_control_on(void) {
    fc.control.set_S2_PITCH_DOWN();
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_CONTROL_ON;
    return 0;
}

extern "C" int event_s2_control_on(void) {
    double S2_rollcmd = S2_ROLLCMD;
    double S2_yawcmd = S2_YAWCMD;
    fc.control.set_attcmd(S2_rollcmd, -5.0, S2_yawcmd);
    return 0;
}

extern "C" int event_aoac_on(void) {
    fc.control.set_S2_AOA();
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_AOAC_ON;
    return 0;
}

extern "C" int event_control_off(void) {
    fc.control.set_NO_CONTROL();
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_CONTROL_OFF;
    return 0;
}

extern "C" int event_s3_control_on(void) {
    double S3_mdot = S3_MDOT;
    double S3_fmass0 = S3_FMASS0;
    double S3_xcg_1 = S3_XCG_1;
    double S3_xcg_0 = S3_XCG_0;
    double S3_isp = S3_ISP;
    double S3_moi_roll_0 = S3_MOI_ROLL_0;
    double S3_moi_roll_1 = S3_MOI_ROLL_1;
    double S3_moi_pitch_0 = S3_MOI_PITCH_0;
    double S3_moi_pitch_1 = S3_MOI_PITCH_1;
    double S3_moi_yaw_0 = S3_MOI_YAW_0;
    double S3_moi_yaw_1 = S3_MOI_YAW_1;
    double S3_kpp = S3_KPP;
    double S3_kpi = S3_KPI;
    double S3_kpd = S3_KPD;
    double S3_kppp = S3_KPPP;
    double S3_pN = S3_PN;
    double S3_krp = S3_KRP;
    double S3_kri = S3_KRI;
    double S3_krd = S3_KRD;
    double S3_krpp = S3_KRPP;
    double S3_rN = S3_RN;
    double S3_kyp = S3_KYP;
    double S3_kyi = S3_KYI;
    double S3_kyd = S3_KYD;
    double S3_kypp = S3_KYPP;
    double S3_yN = S3_YN;
    double S3_kaoap = S3_KAOAP;
    double S3_kaoai = S3_KAOAI;
    double S3_kaoad = S3_KAOAD;
    double S3_kaoapp = S3_KAOAPP;
    double S3_aoaN = S3_AOAN;
    double S3_rollcmd = S3_ROLLCMD;
    double S3_pitchcmd = S3_PITCHCMD;
    double S3_yawcmd = S3_YAWCMD;
    double S3_aoacmd = S3_AOACMD;
    fc.control.set_controller_var(S3_mdot, S3_fmass0, S3_xcg_1, S3_xcg_0, S3_isp);
    fc.control.set_IBBB0(S3_moi_roll_0, S3_moi_pitch_0, S3_moi_yaw_0);
    fc.control.set_IBBB1(S3_moi_roll_1, S3_moi_pitch_1, S3_moi_yaw_1);
    fc.control.get_control_gain(S3_kpp, S3_kpi, S3_kpd, S3_kppp, S3_pN, S3_krp, S3_kri, S3_krd, S3_krpp, S3_rN, S3_kyp, S3_kyi,
                                S3_kyd, S3_kypp, S3_yN, S3_kaoap, S3_kaoai, S3_kaoad, S3_kaoapp, S3_aoaN);
    fc.control.set_attcmd(S3_rollcmd, S3_pitchcmd, S3_yawcmd);
    fc.control.set_S3_AOA();
    fc.control.set_ierror_zero();
    fc.control.set_aoacmd(S3_aoacmd);
    fc.ctl_tvc_db.mission_event_code = MISSION_EVENT_S3_CONTROL_ON;
    return 0;
}

extern "C" int init_stage2_control(void) {
    /* Control variable Stage2 */
    double S2_mdot = S2_MDOT;
    double S2_fmass0 = S2_FMASS0;
    double S2_xcg_1 = S2_XCG_1;
    double S2_xcg_0 = S2_XCG_0;
    double S2_isp = S2_ISP;
    double S2_moi_roll_0 = S2_MOI_ROLL_0;
    double S2_moi_roll_1 = S2_MOI_ROLL_1;
    double S2_moi_pitch_0 = S2_MOI_PITCH_0;
    double S2_moi_pitch_1 = S2_MOI_PITCH_1;
    double S2_moi_yaw_0 = S2_MOI_YAW_0;
    double S2_moi_yaw_1 = S2_MOI_YAW_1;
    double S2_kpp = S2_KPP;
    double S2_kpi = S2_KPI;
    double S2_kpd = S2_KPD;
    double S2_kppp = S2_KPPP;
    double S2_pN = S2_PN;
    double S2_krp = S2_KRP;
    double S2_kri = S2_KRI;
    double S2_krd = S2_KRD;
    double S2_krpp = S2_KRPP;
    double S2_rN = S2_RN;
    double S2_kyp = S2_KYP;
    double S2_kyi = S2_KYI;
    double S2_kyd = S2_KYD;
    double S2_kypp = S2_KYPP;
    double S2_yN = S2_YN;
    double S2_kaoap = S2_KAOAP;
    double S2_kaoai = S2_KAOAI;
    double S2_kaoad = S2_KAOAD;
    double S2_kaoapp = S2_KAOAPP;
    double S2_aoaN = S2_AOAN;
    double S2_rollcmd = S2_ROLLCMD;
    double S2_pitchcmd = S2_PITCHCMD;
    double S2_yawcmd = S2_YAWCMD;
    double S2_aoacmd = S2_AOACMD;

    fc.control.set_controller_var(S2_mdot, S2_fmass0, S2_xcg_1, S2_xcg_0, S2_isp);
    fc.control.set_IBBB0(S2_moi_roll_0, S2_moi_pitch_0, S2_moi_yaw_0);
    fc.control.set_IBBB1(S2_moi_roll_1, S2_moi_pitch_1, S2_moi_yaw_1);
    //  fc.control.set_kpp(S2_kpp)
    //  fc.control.set_kpi(S2_kpi)
    //  fc.control.set_kppp(S2_kppp)
    //  fc.control.set_krp(S2_krp)
    //  fc.control.set_kri(S2_kri)
    //  fc.control.set_krpp(S2_krpp)
    //  fc.control.set_kyp(S2_kyp)
    //  fc.control.set_kyi(S2_kyi)
    //  fc.control.set_kypp(S2_kypp)
    //  fc.control.set_kaoap(S2_kaoap)
    //  fc.control.set_kaoai(S2_kaoai)
    //  fc.control.set_kaoad(S2_kaoad)
    //  fc.control.set_kaoapp(S2_kaoapp)
    fc.control.set_attcmd(S2_rollcmd, S2_pitchcmd, S2_yawcmd);
    fc.control.set_aoacmd(S2_aoacmd);
    fc.control.get_control_gain(S2_kpp, S2_kpi, S2_kpd, S2_kppp, S2_pN, S2_krp, S2_kri, S2_krd,
                                S2_krpp, S2_rN, S2_kyp, S2_kyi, S2_kyd, S2_kypp, S2_yN, S2_kaoap, S2_kaoai, S2_kaoad, S2_kaoapp, S2_aoaN);
}

extern "C" int init_ins_variable(void) {
    //  Vehicle longitude - deg  module newton
    double lonx = 120.893501;
    //  Vehicle latitude  - deg  module newton
    double latx = 22.138917;
    // Vehicle altitude  - m  module newton
    double alt = 5;
    fc.ins.load_location(lonx, latx, alt);

    //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double phibdx = 0.0;
    //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = 90.0;
    //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    double psibdx = 90.0;
    fc.ins.load_angle(psibdx, phibdx, thtbdx);
    //  Initial angle-of-attack   - deg  module newton
    double alpha0x = 0;
    //  Initial sideslip angle    - deg  module newton
    double beta0x = 0;
    //  Vehicle geographic speed  - m/s  module newton
    double dvbe = 0;
    fc.ins.load_geodetic_velocity(alpha0x, beta0x, dvbe);
    fc.ins.set_ideal();
    //  fc.ins.set_non_ideal();
    uint32_t gpsupdate  = 0;
    fc.ins.set_gps_correction(gpsupdate);
    return 0;
}

extern "C" int init_gps_fc_variable() {
    /* GPS */
    double pr_bias_default[4] = {0, 0, 0, 0};
    double pr_noise_default[4] = {0.25, 0.25, 0.25, 0.25};
    double dr_noise_default[4] = {0.03, 0.03, 0.03, 0.03};
    //  User clock frequency error - m/s MARKOV  module gps
    fc.gps.ucfreq_noise      = 0.1;
    // User clock bias error - m GAUSS  module gps
    fc.gps.ucbias_error      = 0;
    // Pseudo-range bias - m GAUSS  module gps
    memcpy(fc.gps.PR_BIAS, pr_bias_default, sizeof(pr_bias_default));
    //  Pseudo-range noise - m MARKOV  module gps
    memcpy(fc.gps.PR_NOISE, pr_noise_default, sizeof(pr_noise_default));
    //  Delta-range noise - m/s MARKOV  module gps
    memcpy(fc.gps.DR_NOISE, dr_noise_default, sizeof(dr_noise_default));
    //  Factor to modifiy initial P-matrix P(1+factp)=module gps
    double gpsr_factp       = 0;
    //  Init 1sig clock bias error of state cov matrix - m=module gps
    double gpsr_pclockb     = 3;
    //  Init 1sig clock freq error of state cov matrix - m/s=module gps
    double gpsr_pclockf     = 1;
    fc.gps.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf);

    //  Factor to modifiy the Q-matrix Q(1+factq)=module gps
    double gpsr_factq       = 0;
    //  1sig clock bias error of process cov matrix - m=module gps
    double gpsr_qclockb     = 0.5;
    //  1sig clock freq error of process cov matrix - m/s=module gps
    double gpsr_qclockf     = 0.1;
    fc.gps.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf);

    //  User clock correlation time constant - s=module gps
    double gpsr_uctime_cor = 100;
    fc.gps.setup_fundamental_dynamic_matrix(gpsr_uctime_cor);

    //  Init 1sig pos values of state cov matrix - m=module gps
    fc.gps.ppos        = 5;
    //  Init 1sig vel values of state cov matrix - m/s=module gps
    fc.gps.pvel        = 0.2;
    //  1sig pos values of process cov matrix - m=module gps
    fc.gps.qpos        = 0.1;
    //  1sig vel values of process cov matrix - m/s=module gps
    fc.gps.qvel        = 0.01;
    //  1sig pos value of meas cov matrix - m=module gps
    fc.gps.rpos        = 1;
    //  1sig vel value of meas cov matrix - m/s=module gps
    fc.gps.rvel        = 0.1;
    //  Factor to modifiy the R-matrix R(1+factr)=module gps
    fc.gps.factr       = 0;
    return 0;
}


extern "C" int run_me() {
    record_gps_slave();
    // realtime();

    unsigned int Year = 2017;
    unsigned int DOY = 81;
    unsigned int Hour = 2;
    unsigned int Min = 0;
    unsigned int Sec = 0;
    fc.time->load_start_time(Year, DOY, Hour, Min, Sec);

    /* INS */
    init_ins_variable();
    /* GPS */
    init_gps_fc_variable();

    init_stage2_control();

    /* events */
    jit_add_read(0.001, "event_liftoff");
    jit_add_read(0.001, "event_control_rcs_on");
    jit_add_read(12.001, "event_control_on");
    //  jit_add_read(15.001, "event_s2_control_on");
    jit_add_read(82.001, "event_aoac_on");
    jit_add_read(100.001, "event_s3_control_on");
    jit_add_read(200.001, "event_control_off");

    exec_set_terminate_time(200.0);

    return 0;
}
