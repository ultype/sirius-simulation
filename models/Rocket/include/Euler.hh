#ifndef __euler_HH__
#define __euler_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the forces Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Euler.cpp))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include "global_constants.hh"
#include "utility_header.hh"

class Euler {
    public:
        Euler(){};

/***********************************Variables describtion******************************/
        double ppx;     /* *io (d/s)        Body roll angular velocity wrt earth in body axes */
        double qqx;     /* *io (d/s)        Body pitch angular velocity wrt earth in body axes */
        double rrx;     /* *io (d/s)        Body yaw angular velocity wrt earth in body axes */
        double wbeb[3]; /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */
        double wbib[3]; /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */
        double wbibd[3];/* *io (r/s2)      Angular velocity of vehicle wrt inertia in body coord - derivative */
        double wbii[3]; /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */
/**************************************************************************************/
};


#endif // __euler_HH__
