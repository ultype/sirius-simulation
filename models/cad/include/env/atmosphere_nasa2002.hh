#ifndef __Atmosphere_nasa2002_HH__
#define __Atmosphere_nasa2002_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (US 1976 Standard Atmosphere (NASA Marshall))
LIBRARY DEPENDENCY:
      ((../../src/env/atmosphere_nasa2002.cpp))
*******************************************************************************/
#include <boost/archive/text_oarchive.hpp>

#include "env/atmosphere.hh"

namespace cad {
    class Atmosphere_nasa2002 : public Atmosphere
    {
        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Atmosphere>(*this);
            };

            Atmosphere_nasa2002();

            virtual ~Atmosphere_nasa2002();

            virtual void set_altitude(double altitude_in_meter);

        private:
            int update_values();
    };
}


#endif//__Atmosphere_nasa2002__
