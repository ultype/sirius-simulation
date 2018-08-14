#include "accel/accelerometer_ideal.hh"

sensor::AccelerometerIdeal::AccelerometerIdeal() {
    snprintf(name, sizeof(name), "Ideal Accelerometer Sensor");
}

void sensor::AccelerometerIdeal::propagate_error(double int_step, struct icf_ctrlblk_t* C) {
    this->FSPCB = grab_FSPB();
    this->EFSPB.zeros();
}
