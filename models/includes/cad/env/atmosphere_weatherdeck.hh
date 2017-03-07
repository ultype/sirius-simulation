#ifndef __Atmosphere_weatherdeck_HH__
#define __Atmosphere_weatherdeck_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (tabular atmosphere from WEATHER_DECK)
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/atmosphere_weatherdeck.cpp))
*******************************************************************************/

#include "cad/env/atmosphere.hh"
#include "cad/datadeck.hh"

namespace cad {
    class Atmosphere_weatherdeck : public Atmosphere
    {
        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Atmosphere>(*this);
            };

            Atmosphere_weatherdeck(char* filepath);

            virtual ~Atmosphere_weatherdeck();

            virtual void set_altitude(double altitude_in_meter);

        private:
            Datadeck weathertable;

            int update_values();
    };
}

#endif//__Atmosphere_weatherdeck__
