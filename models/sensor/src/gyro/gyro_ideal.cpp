#include <armadillo>
#include <cstring>

#include "gyro/gyro_ideal.hh"

sensor::GyroIdeal::GyroIdeal() {
    snprintf(name, sizeof(name), "Ideal Gyro Sensor");
}

void sensor::GyroIdeal::propagate_error(double int_step) {
    this->WBICB = grab_WBIB();
    this->EWBIB.zeros();
    return;
}
