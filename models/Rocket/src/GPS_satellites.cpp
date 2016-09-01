#include "GPS_satellites.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "trick/clock_proto.h"

void GPS_Satellites::default_data(){
    //Setup Orbit related constants
    radius = 26560000;

    //XXX: Store as constant, GM = 3.9860044e14
    velocity = sqrt(3.9860044e14 / pow(radius, 3));
    inclination = 0.95686;
}

void GPS_Satellites::initialize(){
    //Load inputed data to sv_init_data array
    for(int i = 0; i < 24; i ++){
        for(int k = 0; k < 2; k++){
            *(sv_init_data + 2 * i + k) = sv_data[i][k];
        }
    }
}

void GPS_Satellites::propagate_sv(){
    // unpacking the one-dimensional array of SVs into sv_data[24][2]
    for (i = 0; i < 24; i++) {
        for (int k = 0; k < 2; k++) {
            sv_data[i][k] = *(sv_init_data + 2 * i + k);
        }
    }
    // propagating the argument of latitude in time
    for (i = 0; i < 24; i++) {
        sv_data[i][1] = sv_data[i][1] + (almanac_time + (double)(the_clock->clock_time()) / the_clock->clock_tics_per_sec) * velocity;
    }
}
