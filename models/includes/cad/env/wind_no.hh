#ifndef __wind_no_HH__
#define __wind_no_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (constant wind model)
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/wind_no.cpp))
*******************************************************************************/

#include "cad/env/wind.hh"

namespace cad {
    class Wind_No : public Wind
    {
        public:
            Wind_No();

            virtual ~Wind_No();

            virtual void set_altitude(double altitude_in_meter);
    };
}

#endif//__wind_no__
