#ifndef __Rocket_HH__
#define __Rocket_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Rocket Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Rocket.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "aux/utility_header.hh"

class Rocket {
    public:
        Rocket() {}

        void default_data();
        void initialize();

        double IPos[3];     /* *i (m)       Body in Inertial Coordinate */
        double IVel[3];     /* *i (m)       Body in Inertial Coordinate */
        double IW[3];       /* *i (m)       Body in Inertial Coordinate */
};

#endif  // __Rocket_HH__
