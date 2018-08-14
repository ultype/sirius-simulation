#ifndef __ACCEL_IDEAL_HH__
#define __ACCEL_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Accelerometer Implementation)
LIBRARY DEPENDENCY:
      ((../../src/accel/accelerometer_ideal.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "accel/accelerometer.hh"
#include "icf_trx_ctrl.h"
namespace sensor {
class AccelerometerIdeal : public Accelerometer {
    TRICK_INTERFACE(sensor__AccelerometerIdeal);

 public:
    AccelerometerIdeal();

    virtual ~AccelerometerIdeal() {}

    virtual void propagate_error(double int_step, struct icf_ctrlblk_t* C);
};
}  // namespace sensor

#endif  // __ACCEL_IDEAL__
