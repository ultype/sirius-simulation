#include <armadillo>
#include <cstring>

#include "sensor/gyro/gyro_ideal.hh"

sensor::GyroIdeal::GyroIdeal(){
    strcpy(name, "Ideal Gyro Sensor");
}

void sensor::GyroIdeal::propagate_error(double int_step, arma::vec3 WBIB, arma::vec3 FSPB){
    this->WBICB = WBIB;
    return;
}

arma::vec3 sensor::GyroIdeal::get_computed_WBIB(){
    return WBICB;
}
