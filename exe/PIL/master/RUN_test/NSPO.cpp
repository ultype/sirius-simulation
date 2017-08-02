#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"

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

    new_slave->sim_path = std::string(std::getenv("WORKSPACE")) + "/exe/PIL/slave";
    new_slave->S_main_name = "./S_main_Linux_5.4_x86_64.exe";
    new_slave->run_input_file = "RUN_test/NSPO.py";
    new_slave->sync_error_terminate = 1;
    trick_master_slave.master.add_slave(new_slave);
    trick_master_slave.master.enable();
}

extern "C" int event_start() {
    double xcg_0          = 12.032;    //  vehicle initial xcg
    double xcg_1          = 7.965;     //  vehicle final xcg
    double moi_roll_0     = 2426.8;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 914.5;     //  vehicle final moi in roll direction
    double moi_trans_0    = 244537.9;  //  vehicle initial transverse moi
    double moi_trans_1    = 87392.2;   //  vehicle final transverse moi
    double spi            = 255.0;     //  Specific impusle
    double fuel_flow_rate = 88.89;     //  fuel flow rate
    rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate);
    return 0;
}

extern "C" int event_separation_1() {
    rkt.aerodynamics.set_refa(1.13);
    rkt.propulsion.set_aexit(0);
    rkt.aerodynamics.set_xcg_ref(3.429);

    rkt.propulsion.set_vmass0(3893);
    rkt.propulsion.set_fmass0(2963);

    double xcg_0          = 5.829;
    double xcg_1          = 4.683;
    double moi_roll_0     = 615.2;
    double moi_roll_1     = 151.0;
    double moi_trans_0    = 7407.4;
    double moi_trans_1    = 3752.1;
    double spi            = 290;
    double fuel_flow_rate = 29.63;
    rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate);

    rkt.aerodynamics.load_aerotable("../../../auxiliary/aero_table_slv2.txt");

    // speration_2.set_cycle(0.001)
    jit_add_read(350.001, "event_separation_2");

    return 0;
}

extern "C" int event_separation_2() {
    rkt.propulsion.set_vmass0(3863);

    jit_add_event("event_separation_3", "event_separation_3", 0.001);
    return 0;
}

extern "C" int event_separation_3() {
    if (rkt.newton.get_thtvdx() < 3.728) {
        rkt.rcs.set_rcs_thrust(10);
        rkt.aerodynamics.set_xcg_ref(3.2489);
        rkt.rcs.set_roll_mom_max(10);
        rkt.rcs.set_pitch_mom_max(100);
        rkt.rcs.set_yaw_mom_max(20);

        rkt.propulsion.set_vmass0(490);
        rkt.propulsion.set_fmass0(360);

        double xcg_0          = 1.942;
        double xcg_1          = 2.02;
        double moi_roll_0     = 72.7;
        double moi_roll_1     = 61.9;
        double moi_trans_0    = 140.7;
        double moi_trans_1    = 88.8;
        double spi            = 300;
        double fuel_flow_rate = 3.33;
        rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate);

        rkt.aerodynamics.load_aerotable("../../../auxiliary/aero_table_slv1.txt");

        event_manager_get_event("event_separation_3")->deactivate();
        jit_add_event("event_MECO", "event_MECO", 0.05);
    }
    return 0;
}

extern "C" int event_MECO() {
    if (rkt.newton.get_dvbi() > 7613.5) {
        rkt.propulsion.set_no_thrust();
        // rkt.propulsion.set_vmass0(0);

        event_manager_get_event("event_MECO")->deactivate();
    }
    return 0;
}


extern "C" int run_me() {
    record_nspo();
    record_gps();
    // realtime();


    master_startup();

    // Set simulation start time
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    rkt.time->load_start_time(Year, DOY, Hour, Min, Sec);
    rkt.env.dm_RNP();

    // SLV
    double lonx       = 120.893501;  //  Vehicle longitude - deg  module newton
    double latx       = 22.138917;   //  Vehicle latitude  - deg  module newton
    double alt        = 100;         //  Vehicle altitude  - m  module newton
    rkt.newton.load_location(lonx, latx, alt);

    double con_ang = 0.0;
    double con_w = 50.0;
    rkt.newton.load_coning_var(con_ang, con_w);

    double phibdx = 0;       //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = 86.615;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = 90;      //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    rkt.kinematics.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x    = 0;    // Initial angle-of-attack   - deg  module newton
    double beta0x     = 0;    // Initial sideslip angle    - deg  module newton
    double dvbe       = 0;    // Vehicle geographic speed  - m/s  module newton
    rkt.newton.load_geodetic_velocity(alpha0x, beta0x, dvbe);

    rkt.euler.load_angular_velocity(0, 0, 0);

    // environment
    // rkt.env.atmosphere_use_weather_deck("auxiliary/weather_table.txt")
    rkt.env.atmosphere_use_public();
    rkt.env.set_no_wind();
    rkt.env.set_no_wind_turbulunce();
    // aerodynamics
    rkt.aerodynamics.load_aerotable("../../../auxiliary/aero_table_slv3.txt");
    rkt.aerodynamics.set_xcg_ref(9.632);   // Reference cg location from nose - m
    rkt.aerodynamics.set_refa(2.36);       // Reference area for aero coefficients - m^2
    rkt.aerodynamics.set_refd(1.7334);     // Reference length for aero coefficients - m
    rkt.aerodynamics.set_alplimx(20);      // Alpha limiter for vehicle - deg
    rkt.aerodynamics.set_alimitx(5);       // Structural  limiter for vehicle
    // propulsion
    rkt.propulsion.set_vmass0(13970);       // vehicle initial mass
    rkt.propulsion.set_fmass0(8888.9);      // vehicle initail fuel mass

    double xcg_0          = 12.032;    //  vehicle initial xcg
    double xcg_1          = 7.965;     //  vehicle final xcg
    double moi_roll_0     = 2426.8;    //  vehicle initial moi in roll direction
    double moi_roll_1     = 914.5;     //  vehicle final moi in roll direction
    double moi_trans_0    = 244537.9;  //  vehicle initial transverse moi
    double moi_trans_1    = 87392.2;   //  vehicle final transverse moi
    double spi            = 255.0;     //  Specific impusle
    double fuel_flow_rate = 88.89;     //  fuel flow rate
    rkt.propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate);  //  get variable for input file

    rkt.propulsion.set_aexit(0.258242843);  // nozzle exhaust area
    rkt.propulsion.set_payload(96);  // payload mass

    // INS Accel
    // Create a Errorous Accelerometer

    double EMISA[3];      // gauss(0, 1.1e-4)
    double ESCALA[3];      // gauss(0, 2.e-5)
    double EBIASA[3];      // gauss(0, 1.e-6)
    rkt.accelerometer = new sensor::AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt.newton);

    // Create a Ideal Accelerometer
    // rkt.accelerometer = new sensor::AccelerometerIdeal(rkt.newton);

    // INS gyro
    // Create a Errorous Gyro

    double EMISG[3];      // gauss(0, 1.1e-4)
    double ESCALG[3];      // gauss(0, 2.e-5)
    double EBIASG[3];      // gauss(0, 1.e-6)
    rkt.gyro = new sensor::GyroRocket6G(EMISG, ESCALG, EBIASG, rkt.newton, rkt.euler, rkt.kinematics);

    // Create a Ideal Gyro
    // rkt.gyro = new sensor::GyroIdeal(rkt.euler);

    rkt.sdt = new SDT_NONIDEAL();
    // rkt.sdt = new SDT_ideal();

    rkt.rcs.set_roll_mom_max(100);      // RCS rolling moment max value - Nm  module rcs
    rkt.rcs.set_pitch_mom_max(200000);  // RCS pitching moment max value - Nm  module rcs
    rkt.rcs.set_yaw_mom_max(200000);    // RCS yawing moment max value - Nm  module rcs

    double dead_zone = 0.1;                // Dead zone of Schmitt trigger - deg  module rcs
    double hysteresis = 0.1;               // Hysteresis of Schmitt trigger - deg  module rcs
    rkt.rcs.setup_rcs_schmitt_trigger(dead_zone, hysteresis);

    rkt.rcs.set_rcs_thrust(100);       // rcs thrust - N  module rcs
    rkt.rcs.set_rcs_pos(1.66507);       // rcs thruster's postion from nose - m  module rcs
    rkt.rcs.set_rocket_r(0.68);         // rocket's radius - m  module rcs

    rkt.gps_con.readfile("../../../auxiliary/brdc0810.17n");

    /* events */
    jit_add_read(180.001, "event_start");
    jit_add_read(281.001, "event_separation_1");

    exec_set_terminate_time(880.0);

    return 0;
}
