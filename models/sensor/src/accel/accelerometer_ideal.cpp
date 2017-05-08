#include "accel/accelerometer_ideal.hh"

sensor::AccelerometerIdeal::AccelerometerIdeal(Newton &newt)
    :   newton(&newt) {
    snprintf(name, sizeof(name), "Ideal Accelerometer Sensor");
}

void sensor::AccelerometerIdeal::propagate_error(double int_step) {
    this->FSPCB = newton->get_FSPB();
    this->EFSPB.zeros();
}
