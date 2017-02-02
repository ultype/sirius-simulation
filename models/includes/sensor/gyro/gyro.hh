#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

namespace sensor {
    class Gyro
    {
        TRICK_INTERFACE(sensor__Gyro);

        public:
            char name[256];

            Gyro() {};

            virtual ~Gyro() {};

            virtual void propagate_error(double int_step, arma::vec3 WBIB, arma::vec3 FSPB) {};

            virtual arma::vec3 get_computed_WBIB() {};
    };
}

#endif//__GYRO__
