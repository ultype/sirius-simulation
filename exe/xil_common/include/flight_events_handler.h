#ifndef EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
#define EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
#include "sirius_utility.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern Rocket_SimObject rkt;
const double LONX           = 120.8901527777778;  //  Vehicle longitude - deg  module newton
const double LATX           = 22.262097222222224;   //  Vehicle latitude  - deg  module newton
const double ALT            = 6.0;         //  Vehicle altitude  - m  module newton
const double CON_ANG        = 0.0;
const double CON_W          = 50.0;
const double PHIBDX         = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX         = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX         = 90.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X        = 0;    // Initial angle-of-attack   - deg  module newton
const double BETA0X         = 0;    // Initial sideslip angle    - deg  module newton
const double DVBE           = 0;    // Vehicle geographic speed  - m/s  module newton
const double S2_XCG_0          = 6.9903;    //  vehicle initial xcg  TWD use 6.18
const double S2_XCG_1          = 5.5404;     //  vehicle final xcg  TWD use 4.51
const double S2_MOI_ROLL_0     = 1003.315;    //  vehicle initial moi in roll direction  TWD use 461.87
const double S2_MOI_ROLL_1     = 304.448;     //  vehicle final moi in roll direction   TWD use 168.01
const double S2_MOI_PITCH_0    = 22328.316;  //  vehicle initial transverse moi   TWD use 27023.19
const double S2_MOI_PITCH_1    = 14017.096;   //  vehicle final transverse moi    TWD use 18620.51
const double S2_MOI_YAW_0      = 22326.832;  //  vehicle initial transverse moi   TWD use 27023.19
const double S2_MOI_YAW_1      = 14015.971;   //  vehicle final transverse moi    TWD use 18620.51
const double S2_SPI            = 272.0;     //  Specific impusle
const double S2_FUEL_FLOW_RATE = 0.0;     //  fuel flow rate
const double S2_STRUCTURE_MASS = 569.7;
const double S2_PROPELLANT_MASS = 2782.0;  // 2781
const double S2_REMAINING_FUEL_MASS = 145.9761;
const double S3_XCG_0          = 3.017525;    //  vehicle initial xcg  TWD use 6.18
const double S3_XCG_1          = 2.6442;     //  vehicle final xcg  TWD use 4.51
const double S3_MOI_ROLL_0     = 72.939;    //  vehicle initial moi in roll direction  TWD use 461.87
const double S3_MOI_ROLL_1     = 27.41;     //  vehicle final moi in roll direction   TWD use 168.01
const double S3_MOI_PITCH_0    = 530.607;  //  vehicle initial transverse moi   TWD use 27023.19
const double S3_MOI_PITCH_1    = 293.92;   //  vehicle final transverse moi    TWD use 18620.51
const double S3_MOI_YAW_0      = 529.535;  //  vehicle initial transverse moi   TWD use 27023.19
const double S3_MOI_YAW_1      = 292.841;   //  vehicle final transverse moi    TWD use 18620.51
const double S3_SPI            = 292.0;     //  Specific impusle
const double S3_FUEL_FLOW_RATE = 0.0;     //  fuel flow rate
const double S3_STRUCTURE_MASS = 149.4;
const double S3_PROPELLANT_MASS = 409.6;  // 409.6
const double S3_REMAINING_FUEL_MASS = 26.4547;
const double S2_RBODY_XCG_0 = 6.18;
const double S2_RBODY_XCG_1 = 4.51;
const double S2_RBODY_MOI_ROLL_0 = 461.87;
const double S2_RBODY_MOI_ROLL_1 = 168.01;
const double S2_RBODY_MOI_PITCH_0 = 27023.19;
const double S2_RBODY_MOI_PITCH_1 = 18620.51;
const double S2_RBODY_MOI_YAW_0 = 27023.19;
const double S2_RBODY_MOI_YAW_1 = 18620.51;
const double FARING_MASS = 32.56;
const double S3_FARING_SEP_XCG_0 = 3.09199;
const double S3_FARING_SEP_XCG_1 = 2.75567;
const double PAYLOAD = 200.0;
const double S3_vmass0 = PAYLOAD + FARING_MASS + S3_REMAINING_FUEL_MASS + S3_PROPELLANT_MASS + S3_STRUCTURE_MASS;
const double S2_vmass0 = S3_vmass0 + S2_REMAINING_FUEL_MASS + S2_PROPELLANT_MASS + S2_STRUCTURE_MASS;
const double S3_FARING_MOI_ROLL_0 = 63.833;
const double S3_FARING_MOI_ROLL_1 = 20.304;
const double S3_FARING_MOI_PITCH_0 = 419.074;
const double S3_FARING_MOI_PITCH_1 = 218.847;
const double S3_FARING_MOI_YAW_0 = 417.744;
const double S3_FARING_MOI_YAW_1 = 217.509;
/****************Engine coefficients*********************/
const double S2_E1_MASS_0   = 117.13;
const double S2_E1_MASS_1   = 27.42;
const double S2_E1_ROLL_0   = 1.43;
const double S2_E1_ROLL_1   = 0.41;
const double S2_E1_PITCH_0  = 70.88;
const double S2_E1_PITCH_1  = 23.72;
const double S2_E1_YAW_0    = 70.88;
const double S2_E1_YAW_1    = 23.72;
const double S2_E1_XCG_0    = 0.67362;
const double S2_E1_XCG_1    = 0.77217;
/********************************************************/
const double S2_E2_MASS_0   = 117.13;
const double S2_E2_MASS_1   = 27.42;
const double S2_E2_ROLL_0   = 1.43;
const double S2_E2_ROLL_1   = 0.41;
const double S2_E2_PITCH_0  = 70.88;
const double S2_E2_PITCH_1  = 23.72;
const double S2_E2_YAW_0    = 70.88;
const double S2_E2_YAW_1    = 23.72;
const double S2_E2_XCG_0    = 0.67362;
const double S2_E2_XCG_1    = 0.77217;
/********************************************************/
const double S2_E3_MASS_0   = 117.13;
const double S2_E3_MASS_1   = 27.42;
const double S2_E3_ROLL_0   = 1.43;
const double S2_E3_ROLL_1   = 0.41;
const double S2_E3_PITCH_0  = 70.88;
const double S2_E3_PITCH_1  = 23.72;
const double S2_E3_YAW_0    = 70.88;
const double S2_E3_YAW_1    = 23.72;
const double S2_E3_XCG_0    = 0.67362;
const double S2_E3_XCG_1    = 0.77217;
/********************************************************/
const double S2_E4_MASS_0   = 117.13;
const double S2_E4_MASS_1   = 27.42;
const double S2_E4_ROLL_0   = 1.43;
const double S2_E4_ROLL_1   = 0.41;
const double S2_E4_PITCH_0  = 70.88;
const double S2_E4_PITCH_1  = 23.72;
const double S2_E4_YAW_0    = 70.88;
const double S2_E4_YAW_1    = 23.72;
const double S2_E4_XCG_0    = 0.67362;
const double S2_E4_XCG_1    = 0.77217;

extern "C" int event_start() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_LIFTOFF, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);
    rkt.propulsion.engine_ignition();
    rkt.propulsion.set_ignition_time();
    rkt.tvc.set_S2_TVC();
    rkt.tvc.set_s2_tvc_d(0.425);
    rkt.forces.set_e1_d(0.0, 0.0, -0.425);
    rkt.forces.set_e2_d(0.0, 0.425, 0.0);
    rkt.forces.set_e3_d(0.0, 0.0, 0.425);
    rkt.forces.set_e4_d(0.0, -0.425, 0.0);
    return 0;
}

extern "C" int event_hot_staging() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_HOT_STAGING, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_HOT_STAGING);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);
    rkt.propulsion.set_HOT_STAGE();
}

extern "C" int event_separation_1() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_S3_SEPERATION, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S3_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);

    rkt.aerodynamics.set_refa(0.8659);
    rkt.aerodynamics.set_refd(1.05);
    rkt.aerodynamics.load_aerotable("../../../auxiliary/Aero_20180629_S3.txt");

    rkt.propulsion.set_aexit(0.040115);
    rkt.propulsion.set_vmass0(S3_vmass0);
    rkt.propulsion.set_fmass0(S3_PROPELLANT_MASS);
    rkt.propulsion.get_input_file_var(S3_XCG_0, S3_XCG_1, S3_MOI_ROLL_0, S3_MOI_ROLL_1, S3_MOI_PITCH_0, S3_MOI_PITCH_1, S3_MOI_YAW_0, S3_MOI_YAW_1, S3_SPI, S3_FUEL_FLOW_RATE);
    // rkt.propulsion.set_no_thrust();
    rkt.propulsion.set_stage_3();

    rkt.forces.set_reference_point(-3.917);  // set reference point
    rkt.dynamics.set_reference_point(-3.917);
    rkt.tvc.set_S3_reference_p(-3.917);

    rkt.tvc.set_s3_tau1(20.0);
    rkt.tvc.set_s3_tau2(20.0);
    rkt.tvc.set_s3_tau3(20.0);
    rkt.tvc.set_s3_tau4(20.0);
    rkt.tvc.set_s3_ratelim(16.0 * RAD);
    rkt.tvc.set_s3_tvclim(7 * RAD);
    rkt.tvc.set_s3_tvc_acc_lim(360.0 * RAD);
    rkt.tvc.set_S3_TVC();
    rkt.propulsion.engine_ignition();

    return 0;
}

extern "C" int event_S3_ignition() {
    rkt.propulsion.engine_ignition();
}

extern "C" int event_fairing_separation() {
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_FAIRING_JETTSION, rkt.egse_flight_event_handler_bitmap, rkt.flight_event_code_record))
        return 0;
    rkt.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_FAIRING_JETTSION);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", rkt.flight_event_code_record);
    rkt.propulsion.set_faring_sep();
    rkt.propulsion.get_input_file_var(S3_FARING_SEP_XCG_0, S3_FARING_SEP_XCG_1, S3_FARING_MOI_ROLL_0, S3_FARING_MOI_ROLL_1, S3_FARING_MOI_PITCH_0, S3_FARING_MOI_PITCH_1, S3_FARING_MOI_YAW_0, S3_FARING_MOI_YAW_1, S3_SPI, S3_FUEL_FLOW_RATE);
    return 0;
}

extern "C" void master_startup(Rocket_SimObject *rkt) {
    rkt->egse_flight_event_handler_bitmap &= ~(0x1U << 0);
}

extern "C" int master_model_configuration(Rocket_SimObject *rkt) {
    rkt->forces.set_Slosh_flag(0);
    rkt->forces.set_DOF(6);
    rkt->forces.set_damping_ratio(0.005);
    rkt->propulsion.set_CG_OFFSET(0);
    rkt->propulsion.set_TWD(0);
    rkt->forces.set_TWD_flag(0);
    rkt->forces.set_aero_flag(1);
    rkt->dynamics.set_liftoff(0);  // 1 only for test
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
    rkt->aerodynamics.load_aerotable("../../../auxiliary/Aero_20180629_S2+S3.txt");
    rkt->aerodynamics.set_refa(1.65046);       // Reference area for aero coefficients - m^2
    rkt->aerodynamics.set_refd(1.45);     // Reference length for aero coefficients - m
    /********************************************************************************************************/
}

extern "C" void master_init_propulsion(Rocket_SimObject *rkt) {
    /******************************propulsion & mass property***************************************************************************/
    rkt->propulsion.set_vmass0(S2_vmass0);       // vehicle initial mass
    rkt->propulsion.set_fmass0(S2_PROPELLANT_MASS);      // vehicle initail fuel mass
    rkt->propulsion.set_S2_structure_mass(S2_STRUCTURE_MASS);
    rkt->propulsion.set_S2_propellant_mass(S2_PROPELLANT_MASS);
    rkt->propulsion.set_S2_remaining_fuel_mass(S2_REMAINING_FUEL_MASS);
    rkt->propulsion.set_S3_structure_mass(S3_STRUCTURE_MASS);
    rkt->propulsion.set_S3_propellant_mass(S3_PROPELLANT_MASS);
    rkt->propulsion.set_S3_remaining_fuel_mass(S3_REMAINING_FUEL_MASS);
    rkt->propulsion.set_faring_mass(FARING_MASS);
    rkt->propulsion.set_S2_spi(S2_SPI);
    rkt->propulsion.set_S3_spi(S3_SPI);

    rkt->propulsion.load_proptable("../../../auxiliary/Prop_0521_S2+S3.txt");
    rkt->propulsion.get_input_file_var(S2_XCG_0, S2_XCG_1, S2_MOI_ROLL_0, S2_MOI_ROLL_1, S2_MOI_PITCH_0, S2_MOI_PITCH_1, S2_MOI_YAW_0, S2_MOI_YAW_1, S2_SPI, S2_FUEL_FLOW_RATE);
    // rkt->propulsion.get_input_file_var(S2_RBODY_XCG_0, S2_RBODY_XCG_1, S2_RBODY_MOI_ROLL_0, S2_RBODY_MOI_ROLL_1, S2_RBODY_MOI_PITCH_0, S2_RBODY_MOI_PITCH_1, S2_RBODY_MOI_YAW_0, S2_RBODY_MOI_YAW_1, S2_SPI, S2_FUEL_FLOW_RATE);
    rkt->propulsion.set_aexit(0.03333 * 4.0);  // nozzle exhaust area
    rkt->propulsion.set_payload(PAYLOAD);  // payload mass
    rkt->forces.set_reference_point(-8.55);  // set reference point
    rkt->dynamics.set_reference_point(-8.55);
    rkt->tvc.set_S2_reference_p(-8.55);
    rkt->propulsion.set_stage_2();
    rkt->propulsion.set_no_thrust();

    // rkt->propulsion.set_S2_E1_VARIABLE(S2_E1_XCG_0, S2_E1_XCG_1, S2_E1_ROLL_0, S2_E1_ROLL_1, S2_E1_PITCH_0, S2_E1_PITCH_1
    //     , S2_E1_YAW_0, S2_E1_YAW_1, S2_E1_MASS_0, S2_E1_MASS_1);
    // rkt->propulsion.set_S2_E2_VARIABLE(S2_E2_XCG_0, S2_E2_XCG_1, S2_E2_ROLL_0, S2_E2_ROLL_1, S2_E2_PITCH_0, S2_E2_PITCH_1
    //     , S2_E2_YAW_0, S2_E2_YAW_1, S2_E2_MASS_0, S2_E2_MASS_1);
    // rkt->propulsion.set_S2_E3_VARIABLE(S2_E3_XCG_0, S2_E3_XCG_1, S2_E3_ROLL_0, S2_E3_ROLL_1, S2_E3_PITCH_0, S2_E3_PITCH_1
    //     , S2_E3_YAW_0, S2_E3_YAW_1, S2_E3_MASS_0, S2_E3_MASS_1);
    // rkt->propulsion.set_S2_E4_VARIABLE(S2_E4_XCG_0, S2_E4_XCG_1, S2_E4_ROLL_0, S2_E4_ROLL_1, S2_E4_PITCH_0, S2_E4_PITCH_1
    //     , S2_E4_YAW_0, S2_E4_YAW_1, S2_E4_MASS_0, S2_E4_MASS_1);
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
    rkt->tvc.set_s2_tvc_acc_lim(360.0 * RAD);
}

extern "C" void flight_events_handler_configuration(Rocket_SimObject *rkt) {
    /* events */
    jit_add_event("event_start", "LIFTOFF", 0.005);
    jit_add_event("event_separation_1", "S3", 0.005);
    // jit_add_read(102.051 + rkt->stand_still_time, "event_S3_ignition");
    // jit_add_read(107.001, "event_fairing_separation");
    jit_add_event("event_fairing_separation", "FAIRING_JETTSION", 0.005);
    jit_add_event("event_hot_staging", "HOT_STAGING", 0.005);
    exec_set_terminate_time(350.001  + rkt->stand_still_time);
}

#endif  //  EXE_XIL_COMMON_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
