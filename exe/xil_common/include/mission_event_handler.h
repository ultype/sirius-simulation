#ifndef EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
#define EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
#include "sirius_utility.h"

extern "C" void master_startup() {
    rkt.egse_mission_handler_bitmap &= ~(0x1U << 0);
}

extern "C" int event_start() {
    double xcg_0          = 6.4138;    //  vehicle initial xcg
    double xcg_1          = 4.7888;     //  vehicle final xcg
    double moi_roll_0     = 517.8;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 180.9;     //  vehicle final moi in roll direction
    double moi_pitch_0    = 32525.4;  //  vehicle initial transverse moi
    double moi_pitch_1    = 19377.7;   //  vehicle final transverse moi
    double moi_yaw_0    = 32519.8;  //  vehicle initial transverse moi
    double moi_yaw_1    = 19372.3;   //  vehicle final transverse moi
    double spi            = 291.6145604;     //  Specific impusle 291.6145604 274.8
    double fuel_flow_rate = 29.587;     //  fuel flow rate
    if (!IS_MISSION_ARRIVED(MISSION_EVENT_CODE_LIFTOFF, rkt.egse_mission_handler_bitmap, rkt.mission_event_code_record))
        return 0;
    rkt.egse_mission_handler_bitmap &= ~(0x1U << MISSION_EVENT_CODE_LIFTOFF);
    fprintf(stderr, "[Event_start:%f] mission_event_code = %d\n", rkt.mission_event_code_record, exec_get_sim_time());
    rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);
    rkt.tvc.set_S2_TVC();
    return 0;
}

extern "C" int event_separation_1() {
    double xcg_0          = 2.5808;
    double xcg_1          = 2.5371;
    double moi_roll_0     = 65.3;
    double moi_roll_1     = 50.8;
    double moi_pitch_0    = 633.6;
    double moi_pitch_1    = 421.8;
    double moi_yaw_0    = 628.0;
    double moi_yaw_1    = 419.0;
    double spi            = 288.4111169;  // 288.4111169 290.0
    double fuel_flow_rate = 3.814;

     if (!IS_MISSION_ARRIVED(MISSION_EVENT_CODE_S3_CONTROL_ON, rkt.egse_mission_handler_bitmap, rkt.mission_event_code_record))
        return 0;
    rkt.egse_mission_handler_bitmap &= ~(0x1U << MISSION_EVENT_CODE_S3_CONTROL_ON);
    fprintf(stderr, "[Event_start:%f] mission_event_code = %d\n", rkt.mission_event_code_record, exec_get_sim_time());

    rkt.aerodynamics.set_refa(0.7542);
    rkt.aerodynamics.set_refd(0.98);
    rkt.aerodynamics.load_aerotable("../../../auxiliary/Aero_0721_S3.txt");

    rkt.propulsion.set_aexit(0);
    rkt.propulsion.set_vmass0(721.4);
    rkt.propulsion.set_fmass0(381.4);
    rkt.propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);
    rkt.propulsion.set_no_thrust();

    rkt.forces.set_reference_point(-3.275);  // set reference point

    rkt.tvc.set_s3_tau1(20.0);
    rkt.tvc.set_s3_tau2(20.0);
    rkt.tvc.set_s3_tau3(20.0);
    rkt.tvc.set_s3_tau4(20.0);
    rkt.tvc.set_s3_ratelim(16.0 * RAD);
    rkt.tvc.set_s3_tvclim(7 * RAD);
    rkt.tvc.set_S3_TVC();

    return 0;
}

extern "C" int event_S3_ignition() {
    rkt.propulsion.engine_ignition();
    fprintf(stderr, "[Event_start:%f] mission_event_code = %d\n", MISSION_EVENT_CODE_S3_IGNITION, exec_get_sim_time());
}

extern "C" int event_fairing_separation() {
    if (!IS_MISSION_ARRIVED(MISSION_EVENT_FAIRING_JETTSION, rkt.egse_mission_handler_bitmap, rkt.mission_event_code_record))
        return 0;
    rkt.egse_mission_handler_bitmap &= ~(0x1U << MISSION_EVENT_FAIRING_JETTSION);
    fprintf(stderr, "[Event_start:%f] mission_event_code = %d\n", rkt.mission_event_code_record, exec_get_sim_time());
    rkt.propulsion.set_vmass0(691.4);
    return 0;
}
#endif  //  EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
