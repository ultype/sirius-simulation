#ifndef PLANET_HH
#define PLANET_HH
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the lovely mother Earth)
LIBRARY DEPENDENCY:
    ((../src/Earth.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

class Earth {
    public:
        Earth() {};

        const static double radius;  /* (m)        Mean radius of Earth */
        const static double mass;    /* (kg)       Earth Mass */
};

#endif
