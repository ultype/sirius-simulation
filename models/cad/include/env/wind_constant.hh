#ifndef __wind_constant_HH__
#define __wind_constant_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (constant wind model)
LIBRARY DEPENDENCY:
      ((../../src/env/wind_constant.cpp))
*******************************************************************************/

#include "env/wind.hh"

namespace cad {
class Wind_Constant : public Wind {
 public:
    Wind_Constant(double dvba, double dir, double twind, double vertical_wind);

    virtual ~Wind_Constant();

    virtual void set_altitude(double altitude_in_meter);
};
}  // namespace cad

#endif  // __wind_constant__
