#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

#include "../../../xil_common/Modified_data/golden.h"
#include "../../../xil_common/Modified_data/nspo.h"
#include "../../../xil_common/include/realtime.h"
#include "../../../xil_common/include/sirius_utility.h"
#include "../../../xil_common/include/mission_event_handler.h"
#include "../../../xil_common/Modified_data/gps.h"
#include "../../../models/gnc/include/DM_FSW_Interface.hh"

extern "C" int run_me() {
    record_nspo();
    record_gps();
    record_golden();
    external_clock_switch(&rkt.ext_clk);
    realtime();


    master_startup();
    rkt.forces.set_Slosh_flag(0);
    rkt.forces.set_DOF(6);

    /********************************Set simulation start time*****************************************************/
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    rkt.time->load_start_time(Year, DOY, Hour, Min, Sec);
    /***************************************************************************************************************/
    exec_set_time_tic_value(1000000);
    fprintf(stderr, "time_tic_value = %d tics per seconds\n", exec_get_time_tic_value());
    fprintf(stderr, "software_frame = %lf second per frame.\n", exec_get_software_frame());
    fprintf(stderr, "software_frame_tics = %lld tics per_frame\n", exec_get_software_frame_tics());
    /***************************************environment*************************************************************/
    rkt.env.dm_RNP();
    // rkt.env.atmosphere_use_weather_deck("../../../auxiliary/weather_table.txt");
    // rkt.env.atmosphere_use_public();
    rkt.env.atmosphere_use_nasa();
    rkt.env.set_no_wind();
    rkt.env.set_no_wind_turbulunce();
    // rkt.env.set_constant_wind(3.0, 0.0, 1.0, 0.0);
    // rkt.env.set_wind_turbulunce(0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    rkt.gps_con.readfile("../../../auxiliary/brdc0810.17n");
    /****************************************************************************************************************/

    /****************************************SLV************************************************************************/
    double lonx       = 120.893501;  //  Vehicle longitude - deg  module newton
    double latx       = 22.138917;   //  Vehicle latitude  - deg  module newton
    double alt        = 5.0;         //  Vehicle altitude  - m  module newton
    rkt.dynamics.load_location(lonx, latx, alt);

    double con_ang = 0.0;
    double con_w = 50.0;
    rkt.dynamics.load_coning_var(con_ang, con_w);

    double phibdx = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = 90.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    rkt.dynamics.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x    = 0;    // Initial angle-of-attack   - deg  module newton
    double beta0x     = 0;    // Initial sideslip angle    - deg  module newton
    double dvbe       = 0;    // Vehicle geographic speed  - m/s  module newton
    rkt.dynamics.load_geodetic_velocity(alpha0x, beta0x, dvbe);

    rkt.dynamics.load_angular_velocity(0, 0, 0);
    /************************************aerodynamics*******************************************************/
    rkt.aerodynamics.load_aerotable("../../../auxiliary/Aero_0721_S2+S3.txt");
    rkt.aerodynamics.set_refa(1.1309);       // Reference area for aero coefficients - m^2
    rkt.aerodynamics.set_refd(1.2);     // Reference length for aero coefficients - m
    /********************************************************************************************************/

    /******************************propulsion & mass property***************************************************************************/
    rkt.propulsion.set_vmass0(4473.5);       // vehicle initial mass
    rkt.propulsion.set_fmass0(2958.7);      // vehicle initail fuel mass

    double xcg_0          = 6.4138;    //  vehicle initial xcg
    double xcg_1          = 4.7888;     //  vehicle final xcg
    double moi_roll_0     = 517.8;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 180.9;     //  vehicle final moi in roll direction
    double moi_pitch_0    = 32525.4;  //  vehicle initial transverse moi
    double moi_pitch_1    = 19377.7;   //  vehicle final transverse moi
    double moi_yaw_0    = 32519.8;  //  vehicle initial transverse moi
    double moi_yaw_1    = 19372.3;   //  vehicle final transverse moi
    double spi            = 291.6145604;     //  Specific impusle
    double fuel_flow_rate = 29.587;     //  fuel flow rate
    rkt.propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);  //  get variable for input file

    rkt.propulsion.set_aexit(0.03329156 * 4.0);  // nozzle exhaust area
    rkt.propulsion.set_payload(100.0);  // payload mass
    rkt.forces.set_reference_point(-8.436);  // set reference point
    /************************************************************************************************************************/

    /**************************************************Sensor****************************************************************/
    // Accelerometer
    double EMISA[3];      // gauss(0, 1.1e-4)
    double ESCALA[3];      // gauss(0, 2.e-5)
    double EBIASA[3];      // gauss(0, 1.e-6)
    // rkt.accelerometer = new sensor::AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt.newton);
    // Create a Ideal Accelerometer
    rkt.accelerometer = new sensor::AccelerometerIdeal();

    // gyro
    double EMISG[3];      // gauss(0, 1.1e-4)
    double ESCALG[3];      // gauss(0, 2.e-5)
    double EBIASG[3];      // gauss(0, 1.e-6)
    // rkt.gyro = new sensor::GyroRocket6G(EMISG, ESCALG, EBIASG, rkt.newton, rkt.euler, rkt.kinematics);

    // Create a Ideal Gyro
    rkt.gyro = new sensor::GyroIdeal();

    // rkt.sdt = new SDT_NONIDEAL();
    rkt.sdt = new SDT_ideal();
    /******************************************************************************************************************************/

    /****************************************************TVC*************************************************************************/
    rkt.tvc.set_s2_tau1(20.0);
    rkt.tvc.set_s2_tau2(20.0);
    rkt.tvc.set_s2_tau3(20.0);
    rkt.tvc.set_s2_tau4(20.0);

    rkt.tvc.set_s2_ratelim(16.0 * RAD);
    rkt.tvc.set_s2_tvclim(7.0 * RAD);
    /**********************************************************************************************************************************/


    /* events */
    jit_add_event("event_start", "LIFTOFF", 0.001);
    jit_add_event("event_separation_1", "S3", 0.001);
    jit_add_read(101.051, "event_S3_ignition");
    // jit_add_read(107.001, "event_fairing_separation");
    jit_add_event("event_fairing_separation", "FAIRING_JETTSION", 0.001);
    exec_set_terminate_time(200.0);

    return 0;
}
