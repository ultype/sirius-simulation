#ifndef __wind_constant_HH__
#define __wind_constant_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (constant wind model)
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/wind_constant.cpp))
*******************************************************************************/

#include "cad/env/wind.hh"

namespace cad {
    class Wind_Constant : public Wind
    {
        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Wind>(*this);
            };

            Wind_Constant(double dvba, double dir, double twind, double vertical_wind);

            virtual ~Wind_Constant();

            virtual void set_altitude(double altitude_in_meter);
    };
}

#endif//__wind_constant__
