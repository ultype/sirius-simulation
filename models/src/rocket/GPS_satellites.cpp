#include "rocket/GPS_satellites.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

#include "aux/global_constants.hh"

void GPS_Satellites::default_data(){
    //Setup Orbit related constants
    radius = 26560000;

    angular_velocity = sqrt(GM / pow(radius, 3));
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
    for (int i = 0; i < 24; i++) {
        for (int k = 0; k < 2; k++) {
            sv_data[i][k] = *(sv_init_data + 2 * i + k);
        }
    }
    // propagating the argument of latitude in time
    time = get_rettime();
    for (int i = 0; i < 24; i++) {
        sv_data[i][1] = sv_data[i][1] + (almanac_time + time) * angular_velocity;
    }
}
