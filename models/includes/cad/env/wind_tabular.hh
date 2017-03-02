#ifndef __wind_tabular_HH__
#define __wind_tabular_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (tabular wind model)
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/wind_tabular.cpp))
*******************************************************************************/

#include "aux/utility_header.hh"
#include "cad/datadeck.hh"
#include "cad/env/wind.hh"

namespace cad {
    class Wind_Tabular : public Wind
    {
        public:
            Wind_Tabular(char* filepath, double twind, double vertical_wind);

            virtual ~Wind_Tabular();

            virtual void set_altitude(double altitude_in_meter);

        private:
            Datadeck weathertable;
    };
}

#endif//__wind_tabular__
