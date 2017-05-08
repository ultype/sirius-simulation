#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Gyro Implementation)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_ideal.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "gyro/gyro.hh"

#include "Euler.hh"

class _Euler_;

namespace sensor {
class GyroIdeal : public Gyro {
    TRICK_INTERFACE(sensor__GyroIdeal);

 public:
    explicit GyroIdeal(_Euler_ &eul);

    virtual ~GyroIdeal() {}

    virtual void propagate_error(double int_step);

 private:
    /* Routing components */
    _Euler_ * euler;
};
}  // namespace sensor

#endif  // __GYRO_IDEAL__
