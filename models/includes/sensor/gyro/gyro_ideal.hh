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

namespace sensor {
    class GyroIdeal : public Gyro
    {
        TRICK_INTERFACE(sensor__GyroIdeal);

        public:
            char name[256];

            GyroIdeal();

            virtual ~GyroIdeal() {};

            virtual void propagate_error(double int_step, arma::vec3 WBIB, arma::vec3 FSPB);

            virtual arma::vec3 get_computed_WBIB();

        private:
            arma::vec3 WBICB; /* ** (--) */
    };
}

#endif//__GYRO_IDEAL__
