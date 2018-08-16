#include "Ins.hh"
#include "Ins_c.h"
#include <iostream>
double loncx = 120.8901527777778;
double latcx = 22.262097222222224;
double altc = 6.0;
//  Rolling  angle of veh wrt geod coord - deg  module kinematics
double phibdcx = 0.0;
//  Pitching angle of veh wrt geod coord - deg  module kinematics
double thtbdcx = 90.0;
//  Yawing   angle of veh wrt geod coord - deg  module kinematics
double psibdcx = 90.0;
//  Initial angle-of-attack   - deg  module newton
double alpha0x = 0;
//  Initial sideslip angle    - deg  module newton
double beta0x = 0;
//  Vehicle geographic speed  - m/s  module newton
double dvbe = 0;
uint32_t gpsupdate  = 0;
unsigned int Year = 2017;
unsigned int DOY = 81;
unsigned int Hour = 2;
unsigned int Min = 0;
unsigned int Sec = 0;

int cpp_init(INS *ins, time_management *time);
int c_init();

int main(int argc, char *argv[]) {
    INS ins;
    time_management *time = time_management::get_instance();
    INS_alloc();
    cpp_init(&ins, time);
    c_init();

    printf("Error X = %f, Error Y = %f, Error Z = %f\n", ins.get_SBIIC()(0) - gsl_vector_get(SBIIC, 0)
            , ins.get_SBIIC()(1) - gsl_vector_get(SBIIC, 1), ins.get_SBIIC()(2) - gsl_vector_get(SBIIC, 2));
    return 0;
}

int cpp_init(INS *ins, time_management *time) {
    time->load_start_time(Year, DOY, Hour, Min, Sec);
    ins->load_location(loncx, latcx, altc);
    ins->load_angle(psibdcx, phibdcx, thtbdcx);
    ins->load_geodetic_velocity(alpha0x, beta0x, dvbe);
    ins->set_ideal();
    //  fc->ins.set_non_ideal();
    ins->set_gps_correction(gpsupdate);
    ins->initialize();

    std::cout << "CPP->SBIIC[0]: " << ins->get_SBIIC()(0) << "SBIIC[1]: " << ins->get_SBIIC()(1) << "SBIIC[2]: " << ins->get_SBIIC()(2) << std::endl;
    return 0;
}

int c_init() {
    load_start_time(Year, DOY, Hour, Min, Sec, &utctime, &gpstime);
    load_location(loncx, latcx, altc);
    load_angle(psibdcx, phibdcx, thtbdcx, gpstime);
    load_geodetic_velocity(alpha0x, beta0x, dvbe);

    INS_init(gpstime);

    printf("C->SBIIC[0]: %f, SBIIC[1]: %f, SBIIC[2]: %f\n", gsl_vector_get(SBIIC, 0), gsl_vector_get(SBIIC, 1), gsl_vector_get(SBIIC, 2));
    return 0;
}
