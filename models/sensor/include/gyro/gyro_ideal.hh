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

namespace sensor {
class GyroIdeal : public Gyro {
    TRICK_INTERFACE(sensor__GyroIdeal);

 public:
    GyroIdeal();

    virtual ~GyroIdeal() {}

    virtual void propagate_error(double int_step);
};
}  // namespace sensor

#endif  // __GYRO_IDEAL__
