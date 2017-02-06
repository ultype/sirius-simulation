#include <armadillo>
#include <cstring>

#include "sensor/gyro/gyro_ideal.hh"

sensor::GyroIdeal::GyroIdeal(_Euler_ &eul)
    :   euler(&eul)
{
    strcpy(name, "Ideal Gyro Sensor");
}

void sensor::GyroIdeal::propagate_error(double int_step){
    this->WBICB = euler->get_WBIB_();
    return;
}
