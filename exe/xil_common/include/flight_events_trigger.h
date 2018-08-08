#ifndef EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
#define EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
#include "sirius_utility.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern FlightComputer_SimObject fc;
/* Stage2 Control Variable Constant */
const double S2_MDOT = 18.54667;
const double S2_FMASS0 = 2782.0;
const double S2_XCG_1 = 5.5404;
const double S2_XCG_0 = 6.9903;
const double S2_ISP = 272.0;
const double S2_MOI_ROLL_0 = 1003.315;
const double S2_MOI_ROLL_1 = 304.448;
const double S2_MOI_PITCH_0 = 22328.316;
const double S2_MOI_PITCH_1 = 14017.096;
const double S2_MOI_YAW_0 = 22326.832;
const double S2_MOI_YAW_1 = 14015.971;
const double S2_KPP = 3.0;
const double S2_KPI = 0.08;
const double S2_KPD = 0.01;
const double S2_KPPP = 9.375;
const double S2_PN = 1000.0;
const double S2_KRP = 3.0;
const double S2_KRI = 0.33;
const double S2_KRD = 0.015;
const double S2_KRPP = 9.375;
const double S2_RN = 1000.0;
const double S2_KYP = 3.0;
const double S2_KYI = 0.08;
const double S2_KYD = 0.01;
const double S2_KYPP = 9.0;
const double S2_YN = 1000.0;
const double S2_KAOAP = 3.0;
const double S2_KAOAI = 0.15;
const double S2_KAOAD = 0.01;
const double S2_KAOAPP = 9.375;
const double S2_AOAN = 1000.0;
const double S2_ROLLCMD = 0.0;
const double S2_PITCHCMD = -1.0;
const double S2_YAWCMD = 0.0;
const double S2_AOACMD = 0.0;
const double S2_REFERENCE_P = -8.55;

/* Stage3 Control Variable Constant*/
const double S3_MDOT = 2.155789474;
const double S3_FMASS0 = 409.6;
const double S3_XCG_1 = 2.6442;
const double S3_XCG_0 = 3.0234;
const double S3_ISP = 292.0;
const double S3_MOI_ROLL_0 = 72.939;
const double S3_MOI_ROLL_1 = 27.41;
const double S3_MOI_PITCH_0 = 530.607;
const double S3_MOI_PITCH_1 = 293.92;
const double S3_MOI_YAW_0 = 529.535;
const double S3_MOI_YAW_1 = 292.481;
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
const double S3_KAOAPP = 4.6875;
const double S3_AOAN = 1000.0;
const double S3_ROLLCMD = 0.0;
const double S3_PITCHCMD = 0.0;
const double S3_YAWCMD = 0.0;
const double S3_AOACMD = 0.0;
const double S3_REFERENCE_P = -3.917;
const double HS_time = 2.0;

const double FARING_MOI_ROLL_0 = 63.833;
const double FARING_MOI_ROLL_1 = 20.304;
const double FARING_MOI_PITCH_0 = 419.074;
const double FARING_MOI_PITCH_1 = 218.847;
const double FARING_MOI_YAW_0 = 417.744;
const double FARING_MOI_YAW_1 = 217.509;
const double FARING_XCG_0 = 3.09199;
const double FARING_XCG_1 = 2.75567;


extern "C" int event_liftoff(void) {
    fc.ins.set_liftoff(1);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_LIFTOFF;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_LIFTOFF", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_s2_control_on(void) {
    fc.control.set_S2_ROLL_CONTROL();
    // fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S2_ROLL_CONTROL;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S2_ROLL_CONTROL);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S2_ROLL_CONTROL", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_pitch_down_phase_1(void) {
    if (fc.ins.get_altc() >= 500.0) {
        if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_PITCH_DOWN_PHASE_I, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_PITCH_DOWN_PHASE_I))
        return 0;
    } else {
        return 0;
    }

    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_PITCH_DOWN_PHASE_I);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_PITCH_DOWN_PHASE_I", FLIGHT_EVENT_PITCH_DOWN_PHASE_I);
    fc.control.set_S2_PITCH_DOWN_I();
    return 0;
}

extern "C" int event_pitch_down_phase_2(void) {
    if (fc.ins.get_altc() >= 2000.0) {
        if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_PITCH_DOWN_PHASE_II, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_PITCH_DOWN_PHASE_II))
        return 0;
    } else {
        return 0;
    }

    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_PITCH_DOWN_PHASE_II);
    double S2_rollcmd = S2_ROLLCMD;
    double S2_yawcmd = S2_YAWCMD;
    fc.control.set_attcmd(S2_rollcmd, -5.5, S2_yawcmd);
    fc.control.set_S2_PITCH_DOWN_II();
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_PITCH_DOWN_PHASE_II", FLIGHT_EVENT_PITCH_DOWN_PHASE_II);
    return 0;
}

extern "C" int event_aoac_on(void) {
    if (fc.ins.get_altc() >= 20000.0 && fc.ins.get_alphacx() <= 1.0) {
        if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_AOA_CONTROL, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_AOA_CONTROL))
        return 0;
    } else {
        return 0;
    }

    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_AOA_CONTROL);

    fc.control.set_S2_AOA();
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_AOA_CONTROL", FLIGHT_EVENT_AOA_CONTROL);
    return 0;
}

extern "C" int event_control_off(void) {
    fc.control.set_NO_CONTROL();
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_CONTROL_OFF;
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_CONTROL_OFF", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_fairing_jettison(void) {
    if (fc.ins.get_altc() >= 95000.0) {
        if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_FAIRING_JETTSION, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_FAIRING_JETTSION))
        return 0;
    } else {
        return 0;
    }

    fc.control.set_IBBB0(FARING_MOI_ROLL_0, FARING_MOI_PITCH_0, FARING_MOI_YAW_0);
    fc.control.set_IBBB1(FARING_MOI_ROLL_1, FARING_MOI_PITCH_1, FARING_MOI_YAW_1);
    fc.control.set_controller_var(S3_MDOT, S3_FMASS0, FARING_XCG_1, FARING_XCG_0, S3_ISP, S3_MDOT * HS_time);

    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_FAIRING_JETTSION);

    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_FAIRING_JETTSION;
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_FAIRING_JETTSION", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_hot_staging(void) {
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_HOT_STAGING;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_HOT_STAGING);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_HOT_STAGING", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_s3_control_on(void) {
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S3_CONTROL_ON;
    fc.control.set_S3_AOA();
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S3_CONTROL_ON);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S3_CONTROL_ON", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_s3_seperation(void) {
    fc.control.set_controller_var(S3_MDOT, S3_FMASS0, S3_XCG_1, S3_XCG_0, S3_ISP, S3_MDOT * HS_time);
    fc.control.set_IBBB0(S3_MOI_ROLL_0, S3_MOI_PITCH_0, S3_MOI_YAW_0);
    fc.control.set_IBBB1(S3_MOI_ROLL_1, S3_MOI_PITCH_1, S3_MOI_YAW_1);
    fc.control.get_control_gain(S3_KPP, S3_KPI, S3_KPD, S3_KPPP, S3_PN, S3_KRP, S3_KRI, S3_KRD, S3_KRPP, S3_RN, S3_KYP, S3_KYI,
                                S3_KYD, S3_KYPP, S3_YN, S3_KAOAP, S3_KAOAI, S3_KAOAD, S3_KAOAPP, S3_AOAN);
    fc.control.set_attcmd(S3_ROLLCMD, S3_PITCHCMD, S3_YAWCMD);
    fc.control.set_aoacmd(S3_AOACMD);
    fc.control.set_ierror_zero();
    fc.control.set_reference_point(S3_REFERENCE_P);
    fc.control.set_engine_d(0.0);
    // fc.control.set_S3_AOA();
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S3_SEPERATION;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S3_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S3_SEPERATION", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int slave_init_stage2_control(FlightComputer_SimObject *fc) {
    /* Control variable Stage2 */
    fc->control.set_controller_var(S2_MDOT, S2_FMASS0, S2_XCG_1, S2_XCG_0, S2_ISP, 0.0);
    fc->control.set_IBBB0(S2_MOI_ROLL_0, S2_MOI_PITCH_0, S2_MOI_YAW_0);
    fc->control.set_IBBB1(S2_MOI_ROLL_1, S2_MOI_PITCH_1, S2_MOI_YAW_1);
    fc->control.set_attcmd(S2_ROLLCMD, S2_PITCHCMD, S2_YAWCMD);
    fc->control.set_aoacmd(S2_AOACMD);
    fc->control.get_control_gain(S2_KPP, S2_KPI, S2_KPD, S2_KPPP, S2_PN, S2_KRP, S2_KRI, S2_KRD,
                                S2_KRPP, S2_RN, S2_KYP, S2_KYI, S2_KYD, S2_KYPP, S2_YN, S2_KAOAP, S2_KAOAI, S2_KAOAD, S2_KAOAPP, S2_AOAN);
    fc->control.set_reference_point(S2_REFERENCE_P);
    fc->control.set_engine_d(0.425);
    return 0;
}

extern "C" int slave_init_ins_variable(FlightComputer_SimObject *fc) {
    //  Vehicle longitude - deg  module newton
    double lonx = 120.8901527777778;
    //  Vehicle latitude  - deg  module newton
    double latx = 22.262097222222224;
    // Vehicle altitude  - m  module newton
    double alt = 6.0;
    fc->ins.load_location(lonx, latx, alt);

    //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double phibdx = 0.0;
    //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = 90.0;
    //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    double psibdx = 90.0;
    fc->ins.load_angle(psibdx, phibdx, thtbdx);
    //  Initial angle-of-attack   - deg  module newton
    double alpha0x = 0;
    //  Initial sideslip angle    - deg  module newton
    double beta0x = 0;
    //  Vehicle geographic speed  - m/s  module newton
    double dvbe = 0;
    fc->ins.load_geodetic_velocity(alpha0x, beta0x, dvbe);
    fc->ins.set_ideal();
    //  fc->ins.set_non_ideal();
    uint32_t gpsupdate  = 0;
    fc->ins.set_gps_correction(gpsupdate);
    return 0;
}

extern "C" int slave_init_gps_fc_variable(FlightComputer_SimObject *fc) {
    /* GPS */
    double pr_bias_default[4] = {0, 0, 0, 0};
    double pr_noise_default[4] = {0.25, 0.25, 0.25, 0.25};
    double dr_noise_default[4] = {0.03, 0.03, 0.03, 0.03};
    //  User clock frequency error - m/s MARKOV  module gps
    fc->gps.ucfreq_noise      = 0.1;
    // User clock bias error - m GAUSS  module gps
    fc->gps.ucbias_error      = 0;
    // Pseudo-range bias - m GAUSS  module gps
    memcpy(fc->gps.PR_BIAS, pr_bias_default, sizeof(pr_bias_default));
    //  Pseudo-range noise - m MARKOV  module gps
    memcpy(fc->gps.PR_NOISE, pr_noise_default, sizeof(pr_noise_default));
    //  Delta-range noise - m/s MARKOV  module gps
    memcpy(fc->gps.DR_NOISE, dr_noise_default, sizeof(dr_noise_default));
    //  Factor to modifiy initial P-matrix P(1+factp)=module gps
    double gpsr_factp       = 0;
    //  Init 1sig clock bias error of state cov matrix - m=module gps
    double gpsr_pclockb     = 3;
    //  Init 1sig clock freq error of state cov matrix - m/s=module gps
    double gpsr_pclockf     = 1;
    fc->gps.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf);

    //  Factor to modifiy the Q-matrix Q(1+factq)=module gps
    double gpsr_factq       = 0;
    //  1sig clock bias error of process cov matrix - m=module gps
    double gpsr_qclockb     = 0.5;
    //  1sig clock freq error of process cov matrix - m/s=module gps
    double gpsr_qclockf     = 0.1;
    fc->gps.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf);

    //  User clock correlation time constant - s=module gps
    double gpsr_uctime_cor = 100;
    fc->gps.setup_fundamental_dynamic_matrix(gpsr_uctime_cor);

    //  Init 1sig pos values of state cov matrix - m=module gps
    fc->gps.ppos        = 5;
    //  Init 1sig vel values of state cov matrix - m/s=module gps
    fc->gps.pvel        = 0.2;
    //  1sig pos values of process cov matrix - m=module gps
    fc->gps.qpos        = 0.1;
    //  1sig vel values of process cov matrix - m/s=module gps
    fc->gps.qvel        = 0.01;
    //  1sig pos value of meas cov matrix - m=module gps
    fc->gps.rpos        = 1;
    //  1sig vel value of meas cov matrix - m/s=module gps
    fc->gps.rvel        = 0.1;
    //  Factor to modifiy the R-matrix R(1+factr)=module gps
    fc->gps.factr       = 0;
    return 0;
}

extern "C" int slave_init_time(FlightComputer_SimObject *fc) {
    unsigned int Year = 2017;
    unsigned int DOY = 81;
    unsigned int Hour = 2;
    unsigned int Min = 0;
    unsigned int Sec = 0;
    fc->time->load_start_time(Year, DOY, Hour, Min, Sec);
    return 0;
}

extern "C" void flight_events_trigger_configuration(FlightComputer_SimObject *fc) {
    /* events */
    jit_add_read(0.001 + fc->stand_still_time, "event_liftoff");
    jit_add_read(0.001 + fc->stand_still_time, "event_s2_control_on");
    jit_add_read(148.0 + fc->stand_still_time, "event_hot_staging");
    jit_add_read(151.0 + fc->stand_still_time, "event_s3_seperation");
    jit_add_read(151.05 + fc->stand_still_time, "event_s3_control_on");
    jit_add_event("event_pitch_down_phase_1", "PITCH DOWN PHASE I", 0.05);
    jit_add_event("event_pitch_down_phase_2", "PITCH DOWN PHASE II", 0.05);
    jit_add_event("event_fairing_jettison", "FARING JETTISON", 0.05);
    jit_add_event("event_aoac_on", "AOA CONTROL", 0.05);
    // jit_add_read(12.001 + fc->stand_still_time, "event_pitch_down_phase_1");
    // // jit_add_read(15.001 + fc->stand_still_time, "event_s2_control_on");
    // jit_add_read(82.001 + fc->stand_still_time, "event_aoac_on");
    // jit_add_read(100.001 + fc->stand_still_time, "event_s3_seperation");
    // jit_add_read(102.051 + fc->stand_still_time, "event_s3_control_on");
    // jit_add_read(107.001 + fc->stand_still_time, "event_fairing_jettison");
    // jit_add_read(200.001 + fc->stand_still_time, "event_control_off");

    exec_set_terminate_time(350.001 + fc->stand_still_time);
}
#endif  //  EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_TRIGGER_H_
