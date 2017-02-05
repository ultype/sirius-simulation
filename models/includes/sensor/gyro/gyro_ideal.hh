#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
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

            virtual arma::vec3 get_computed_WBIB();
            virtual arma::vec3 get_error_of_WBIB();

        private:
            /* Routing components */
            _Euler_ * euler;

            arma::vec3 WBICB; /* ** (--) */
    };
}

#endif//__GYRO_IDEAL__
