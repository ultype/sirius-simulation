#ifndef EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
#define EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
#include "sirius_utility.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern Rocket_SimObject rkt;
const double LONX           = 120.893501;  //  Vehicle longitude - deg  module newton
const double LATX           = 22.138917;   //  Vehicle latitude  - deg  module newton
const double ALT            = 5.0;         //  Vehicle altitude  - m  module newton
const double CON_ANG        = 0.0;
const double CON_W          = 50.0;
const double PHIBDX         = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX         = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX         = 90.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X        = 0;    // Initial angle-of-attack   - deg  module newton
const double BETA0X         = 0;    // Initial sideslip angle    - deg  module newton
const double DVBE           = 0;    // Vehicle geographic speed  - m/s  module newton
const double XCG_0          = 6.4138;    //  vehicle initial xcg
const double XCG_1          = 4.7888;     //  vehicle final xcg
const double MOI_ROLL_0     = 517.8;    //  vehicle initial moi in roll direction
const double MOI_ROLL_1     = 180.9;     //  vehicle final moi in roll direction
const double MOI_PITCH_0    = 32525.4;  //  vehicle initial transverse moi
const double MOI_PITCH_1    = 19377.7;   //  vehicle final transverse moi
const double MOI_YAW_0      = 32519.8;  //  vehicle initial transverse moi
const double MOI_YAW_1      = 19372.3;   //  vehicle final transverse moi
const double SPI            = 291.6145604;     //  Specific impusle
const double FUEL_FLOW_RATE = 29.587;     //  fuel flow rate

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
    PRINT_MISSION_MESSAGE("EGSE", exec_get_sim_time(), "Recived mission_code", rkt.mission_event_code_record);
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

     if (!IS_MISSION_ARRIVED(MISSION_EVENT_CODE_S3_SEPERATION, rkt.egse_mission_handler_bitmap, rkt.mission_event_code_record))
        return 0;
    rkt.egse_mission_handler_bitmap &= ~(0x1U << MISSION_EVENT_CODE_S3_SEPERATION);
    PRINT_MISSION_MESSAGE("EGSE", exec_get_sim_time(), "Recived mission_code", rkt.mission_event_code_record);

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
}

extern "C" int event_fairing_separation() {
    if (!IS_MISSION_ARRIVED(MISSION_EVENT_FAIRING_JETTSION, rkt.egse_mission_handler_bitmap, rkt.mission_event_code_record))
        return 0;
    rkt.egse_mission_handler_bitmap &= ~(0x1U << MISSION_EVENT_FAIRING_JETTSION);
    PRINT_MISSION_MESSAGE("EGSE", exec_get_sim_time(), "Recived mission_code", rkt.mission_event_code_record);
    rkt.propulsion.set_vmass0(691.4);
    return 0;
}

extern "C" void master_startup(Rocket_SimObject *rkt) {
    rkt->egse_mission_handler_bitmap &= ~(0x1U << 0);
}

extern "C" int master_model_configuration(Rocket_SimObject *rkt) {
    rkt->forces.set_Slosh_flag(0);
    rkt->forces.set_DOF(6);
    rkt->forces.set_damping_ratio(0.005);
    rkt->propulsion.set_CG_OFFSET(0);
    // rkt->propulsion.set_TWD(0);
}

extern "C" void master_init_time(Rocket_SimObject *rkt) {
    /********************************Set simulation start time*****************************************************/
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    rkt->time->load_start_time(Year, DOY, Hour, Min, Sec);
}

extern "C" void master_init_environment(Rocket_SimObject *rkt) {
    /***************************************environment*************************************************************/
    rkt->env.dm_RNP();
    // rkt->env.atmosphere_use_weather_deck("../../../auxiliary/weather_table.txt");
    // rkt->env.atmosphere_use_public();
    rkt->env.atmosphere_use_nasa();
    rkt->env.set_no_wind();
    rkt->env.set_no_wind_turbulunce();
    // rkt->env.set_constant_wind(3.0, 0.0, 1.0, 0.0);
    // rkt->env.set_wind_turbulunce(0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    rkt->gps_con.readfile("../../../auxiliary/brdc0810.17n");
}

extern "C" void master_init_slv(Rocket_SimObject *rkt) {
    /****************************************SLV************************************************************************/
    double lonx       = LONX;  //  Vehicle longitude - deg  module newton
    double latx       = LATX;   //  Vehicle latitude  - deg  module newton
    double alt        = ALT;         //  Vehicle altitude  - m  module newton
    rkt->dynamics.load_location(lonx, latx, alt);

    double con_ang = CON_ANG;
    double con_w = CON_W;
    rkt->dynamics.load_coning_var(con_ang, con_w);

    double phibdx = PHIBDX;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = THTBDX;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = PSIBDX;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    rkt->dynamics.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x    = ALPHA0X;   // Initial angle-of-attack   - deg  module newton
    double beta0x     = BETA0X;   // Initial sideslip angle    - deg  module newton
    double dvbe       = DVBE;   // Vehicle geographic speed  - m/s  module newton
    rkt->dynamics.load_geodetic_velocity(alpha0x, beta0x, dvbe);
    rkt->dynamics.load_angular_velocity(0, 0, 0);
}

extern "C" void master_init_aerodynamics(Rocket_SimObject *rkt) {
    /************************************aerodynamics*******************************************************/
    rkt->aerodynamics.load_aerotable("../../../auxiliary/Aero_0721_S2+S3.txt");
    rkt->aerodynamics.set_refa(1.1309);       // Reference area for aero coefficients - m^2
    rkt->aerodynamics.set_refd(1.2);     // Reference length for aero coefficients - m
    /********************************************************************************************************/
}

extern "C" void master_init_propulsion(Rocket_SimObject *rkt) {
    /******************************propulsion & mass property***************************************************************************/
    rkt->propulsion.set_vmass0(4473.5);       // vehicle initial mass
    rkt->propulsion.set_fmass0(2958.7);      // vehicle initail fuel mass

    double xcg_0          = XCG_0;    //  vehicle initial xcg
    double xcg_1          = XCG_1;     //  vehicle final xcg
    double moi_roll_0     = MOI_ROLL_0;    //  vehicle initial moi in roll direction
    double moi_roll_1     = MOI_ROLL_1;     //  vehicle final moi in roll direction
    double moi_pitch_0    = MOI_PITCH_0;  //  vehicle initial transverse moi
    double moi_pitch_1    = MOI_PITCH_1;   //  vehicle final transverse moi
    double moi_yaw_0    = MOI_YAW_0;  //  vehicle initial transverse moi
    double moi_yaw_1    = MOI_YAW_1;   //  vehicle final transverse moi
    double spi            = SPI;     //  Specific impusle
    double fuel_flow_rate = FUEL_FLOW_RATE;     //  fuel flow rate
    rkt->propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);  //  get variable for input file

    rkt->propulsion.set_aexit(0.03329156 * 4.0);  // nozzle exhaust area
    rkt->propulsion.set_payload(0.0);  // payload mass
    rkt->forces.set_reference_point(-8.436);  // set reference point
}

extern "C" void master_init_sensors(Rocket_SimObject *rkt) {
    /**************************************************Sensor****************************************************************/
    // Accelerometer
    double EMISA[3];      // gauss(0, 1.1e-4)
    double ESCALA[3];      // gauss(0, 2.e-5)
    double EBIASA[3];      // gauss(0, 1.e-6)
    // rkt->accelerometer = new sensor::AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt->newton);
    // Create a Ideal Accelerometer
    rkt->accelerometer = new sensor::AccelerometerIdeal();

    // gyro
    double EMISG[3];      // gauss(0, 1.1e-4)
    double ESCALG[3];      // gauss(0, 2.e-5)
    double EBIASG[3];      // gauss(0, 1.e-6)
    // rkt->gyro = new sensor::GyroRocket6G(EMISG, ESCALG, EBIASG, rkt->newton, rkt->euler, rkt->kinematics);

    // Create a Ideal Gyro
    rkt->gyro = new sensor::GyroIdeal();

    // rkt->sdt = new SDT_NONIDEAL();
    rkt->sdt = new SDT_ideal();
}

extern "C" void master_init_tvc(Rocket_SimObject *rkt) {
    /****************************************************TVC*************************************************************************/
    rkt->tvc.set_s2_tau1(20.0);
    rkt->tvc.set_s2_tau2(20.0);
    rkt->tvc.set_s2_tau3(20.0);
    rkt->tvc.set_s2_tau4(20.0);

    rkt->tvc.set_s2_ratelim(16.0 * RAD);
    rkt->tvc.set_s2_tvclim(7.0 * RAD);
}

extern "C" void mission_event_handler_configuration(Rocket_SimObject *rkt) {
    /* events */
    double stand_still_time = 180.0;
    jit_add_event("event_start", "LIFTOFF", rkt->int_step);
    jit_add_event("event_separation_1", "S3", rkt->int_step);
    jit_add_read(102.051 + stand_still_time, "event_S3_ignition");
    // jit_add_read(107.001, "event_fairing_separation");
    jit_add_event("event_fairing_separation", "FAIRING_JETTSION", rkt->int_step);
    exec_set_terminate_time(200.001  + stand_still_time);
}

#endif  //  EXE_XIL_COMMON_INCLUDE_MISSION_EVENT_HANDLER_H_
