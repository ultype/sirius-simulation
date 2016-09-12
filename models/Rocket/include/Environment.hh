#ifndef __environment_HH__
#define __environment_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Environment Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Environment.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "global_constants.hh"
#include "utility_header.hh"


class Environment{

    public:

/***********************************Variables describtion******************************/
        double press;       /* *io (pa)         Atmospheric pressure */
        double rho;         /* *io (kg/m3)      Atmospheric Density */
        double vsound;      /* *io (m/s)        Sonic speed */
        double vmach;       /* *io (--)         Mach number */
        double pdynmc;      /* *io (pa)         Dynamic pressure */
        double tempk;       /* *io (K)          Atmospheric temperature */
        double gravg[3];    /* *io (m/s2)       Gravity acceleration in geocentric coord */
        double grav;        /* *io (m/s2)       Magnitude of gravity acceleration */
        double dvae;        /* *io (m/s)        Madnitude of constant air speed */
        double dvba;        /* *io (m/s)        Vehicle speed wrt air */

        double vaed[3];     /* *io (m/s)        Smoothed wind velocity in geodetic coord */

};

#endif  // __kinematics_HH__
