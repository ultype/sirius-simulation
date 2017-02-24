#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Gyro Implementation)
LIBRARY DEPENDENCY:
      ((../../../src/sensor/gyro/gyro_ideal.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

#include "sensor/gyro/gyro.hh"

#include "rocket/Euler.hh"

namespace sensor {
    class GyroIdeal : public Gyro
    {
        TRICK_INTERFACE(sensor__GyroIdeal);

        public:
            char name[256];

            GyroIdeal(_Euler_ &eul);

            virtual ~GyroIdeal() {};

            virtual void propagate_error(double int_step);

        private:
            /* Routing components */
            _Euler_ * euler;
    };
}

#endif//__GYRO_IDEAL__
