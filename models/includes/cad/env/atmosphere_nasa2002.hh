#ifndef __Atmosphere_nasa2002_HH__
#define __Atmosphere_nasa2002_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (US 1976 Standard Atmosphere (NASA Marshall))
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/atmosphere_nasa2002.cpp))
*******************************************************************************/

#include "cad/env/atmosphere.hh"

namespace cad {
    class Atmosphere_nasa2002 : public Atmosphere
    {
        public:
            Atmosphere_nasa2002();

            virtual ~Atmosphere_nasa2002();

            virtual void set_altitude(double altitude_in_meter);

            virtual double get_temperature_in_kelvin();
            virtual double get_density();
            virtual double get_pressure();
            virtual double get_speed_of_sound();

        private:
            int update_values();
    };
}


#endif//__Atmosphere_nasa2002__
