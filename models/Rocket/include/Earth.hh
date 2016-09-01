#ifndef PLANET_HH
#define PLANET_HH
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the lovely mother Earth)
LIBRARY DEPENDENCY:
    ((../src/Planet.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#define EARTH_MASS 5.9721986e24
#define EARTH_RADIUS 6367500.0

class Earth {
    public:
    Earth();

    static double radius;  /* (m)        Mean radius of Earth */
    static double mass;    /* (kg)       Earth Mass */

};



#endif
