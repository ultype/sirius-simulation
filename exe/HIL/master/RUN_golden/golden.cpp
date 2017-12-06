#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

#include "../../../public/Modified_data/golden.h"
#include "../Modified_data/nspo.h"
#include "../../../public/Modified_data/realtime.h"
#include "../Modified_data/gps.h"

extern "C" void master_startup() {
    Trick::MSSocket *new_connection = new Trick::MSSocket();
    Trick::SlaveInfo *new_slave = new Trick::SlaveInfo();
    new_slave->set_connection_type(new_connection);

    if (std::getenv("WORKSPACE") == NULL) {
        throw std::invalid_argument("Must set $WORKSPACE");
    }

    new_slave->sim_path = std::string(std::getenv("WORKSPACE")) + "/exe/HIL/slave";
    new_slave->S_main_name = "./S_main_Linux_5.4_x86_64.exe";
    new_slave->run_input_file = "RUN_golden/golden.py";
    new_slave->sync_error_terminate = 1;
    trick_master_slave.master.add_slave(new_slave);
    trick_master_slave.master.enable();
}

extern "C" int event_start() {
    double xcg_0          = 6.4138;    //  vehicle initial xcg
    double xcg_1          = 4.7888;     //  vehicle final xcg
    double moi_roll_0     = 180.39;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 114.46;     //  vehicle final moi in roll direction
    double moi_pitch_0    = 15883.31;  //  vehicle initial transverse moi
    double moi_pitch_1    = 7353.05;   //  vehicle final transverse moi
    double moi_yaw_0    = 15881.97;  //  vehicle initial transverse moi
    double moi_yaw_1    = 7351.71;   //  vehicle final transverse moi
    double spi            = 274.8;     //  Specific impusle
    double fuel_flow_rate = 31.3974;     //  fuel flow rate
    rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);
    rkt.tvc.set_S2_TVC();
    return 0;
}

extern "C" int event_separation_1() {
    rkt.aerodynamics.set_refa(0.7542);
    rkt.aerodynamics.set_refd(0.98);
    rkt.aerodynamics.load_aerotable("../../../auxiliary/Aero_0721_S3.txt");

    rkt.propulsion.set_aexit(0);
    rkt.propulsion.set_vmass0(777.18);
    rkt.propulsion.set_fmass0(379.31);

    double xcg_0          = 2.5808;
    double xcg_1          = 2.5371;
    double moi_roll_0     = 26.25;
    double moi_roll_1     = 25.97;
    double moi_pitch_0    = 181.69;
    double moi_pitch_1    = 181.68;
    double moi_yaw_0    = 182.27;
    double moi_yaw_1    = 180.58;
    double spi            = 290;
    double fuel_flow_rate = 3.4931;
    rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);

    rkt.tvc.set_s3_tau1(20.0);
    rkt.tvc.set_s3_tau2(20.0);
    rkt.tvc.set_s3_tau3(20.0);
    rkt.tvc.set_s3_tau4(20.0);
    rkt.tvc.set_s3_ratelim(16.0 * RAD);
    rkt.tvc.set_s3_tvclim(7 * RAD);
    rkt.tvc.set_S3_TVC();

    return 0;
}

extern "C" int run_me() {
    record_nspo();
    record_gps();
    record_golden();
    // realtime();


    master_startup();

    /********************************Set simulation start time*****************************************************/
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    rkt.time->load_start_time(Year, DOY, Hour, Min, Sec);
    /***************************************************************************************************************/
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
    rkt.newton.load_location(lonx, latx, alt);

    double con_ang = 0.0;
    double con_w = 50.0;
    rkt.newton.load_coning_var(con_ang, con_w);

    double phibdx = 0.0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = 90.0;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = 90.0;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    rkt.kinematics.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x    = 0;    // Initial angle-of-attack   - deg  module newton
    double beta0x     = 0;    // Initial sideslip angle    - deg  module newton
    double dvbe       = 0;    // Vehicle geographic speed  - m/s  module newton
    rkt.newton.load_geodetic_velocity(alpha0x, beta0x, dvbe);

    rkt.euler.load_angular_velocity(0, 0, 0);
    /************************************aerodynamics*******************************************************/
    rkt.aerodynamics.load_aerotable("../../../auxiliary/Aero_0721_S2+S3.txt");
    rkt.aerodynamics.set_refa(1.1309);       // Reference area for aero coefficients - m^2
    rkt.aerodynamics.set_refd(1.2);     // Reference length for aero coefficients - m
    /********************************************************************************************************/

    /******************************propulsion & mass property***************************************************************************/
    rkt.propulsion.set_vmass0(4057.14);       // vehicle initial mass
    rkt.propulsion.set_fmass0(3139.74);      // vehicle initail fuel mass

    double xcg_0          = 6.4138;    //  vehicle initial xcg
    double xcg_1          = 4.7888;     //  vehicle final xcg
    double moi_roll_0     = 180.39;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 114.46;     //  vehicle final moi in roll direction
    double moi_pitch_0    = 15883.31;  //  vehicle initial transverse moi
    double moi_pitch_1    = 7353.05;   //  vehicle final transverse moi
    double moi_yaw_0    = 15881.97;  //  vehicle initial transverse moi
    double moi_yaw_1    = 7351.71;   //  vehicle final transverse moi
    double spi            = 274.8;     //  Specific impusle
    double fuel_flow_rate = 31.3974;     //  fuel flow rate
    rkt.propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_pitch_0, moi_pitch_1, moi_yaw_0, moi_yaw_1, spi, fuel_flow_rate);  //  get variable for input file

    rkt.propulsion.set_aexit(0.03329156 * 4.0);  // nozzle exhaust area
    rkt.propulsion.set_payload(0);  // payload mass
    /************************************************************************************************************************/

    /**************************************************Sensor****************************************************************/
    // Accelerometer
    double EMISA[3];      // gauss(0, 1.1e-4)
    double ESCALA[3];      // gauss(0, 2.e-5)
    double EBIASA[3];      // gauss(0, 1.e-6)
    // rkt.accelerometer = new sensor::AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt.newton);
    // Create a Ideal Accelerometer
    rkt.accelerometer = new sensor::AccelerometerIdeal(rkt.newton);

    // gyro
    double EMISG[3];      // gauss(0, 1.1e-4)
    double ESCALG[3];      // gauss(0, 2.e-5)
    double EBIASG[3];      // gauss(0, 1.e-6)
    // rkt.gyro = new sensor::GyroRocket6G(EMISG, ESCALG, EBIASG, rkt.newton, rkt.euler, rkt.kinematics);

    // Create a Ideal Gyro
    rkt.gyro = new sensor::GyroIdeal(rkt.euler);

    // rkt.sdt = new SDT_NONIDEAL();
    rkt.sdt = new SDT_ideal();
    /******************************************************************************************************************************/

    /****************************************************TVC*************************************************************************/
    rkt.tvc.set_s2_tau1(20.0);
    rkt.tvc.set_s2_tau2(20.0);
    rkt.tvc.set_s2_tau3(20.0);
    rkt.tvc.set_s2_tau4(20.0);

    rkt.tvc.set_s2_ratelim(16 * RAD);
    rkt.tvc.set_s2_tvclim(7 * RAD);
    /**********************************************************************************************************************************/


    /* events */
    jit_add_read(0.001, "event_start");
    jit_add_read(100.001, "event_separation_1");

    exec_set_terminate_time(200.0);

    return 0;
}
