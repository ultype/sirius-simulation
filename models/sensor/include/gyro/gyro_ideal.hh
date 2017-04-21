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

#include <boost/archive/text_oarchive.hpp>

class _Euler_;

namespace sensor {
    class GyroIdeal : public Gyro
    {
        TRICK_INTERFACE(sensor__GyroIdeal);

        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Gyro>(*this);

                ar & euler;
            };

            GyroIdeal(_Euler_ &eul);

            virtual ~GyroIdeal() {};

            virtual void propagate_error(double int_step);

        private:
            /* Routing components */
            _Euler_ * euler;
    };
}

#endif//__GYRO_IDEAL__
