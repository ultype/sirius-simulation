#ifndef __forces_HH__
#define __forces_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the forces Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Forces.cpp))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include "global_constants.hh"
#include "utility_header.hh"
#include "Environment.hh"
#include "Rcs.hh"
#include "Tvc.hh"
#include "Kinematics.hh"
#include "Newton.hh"
#include "Control.hh"
class Forces{
    public:
        Forces(){};
    //     void init_force(Environment* env, Propulsion* prop, RCS* rcs
    // , AeroDynamic* aero, TVC* tvc);
    //     void forces();
/***********************************Variables describtion******************************/
        double fapb[3];         /* *io (N)      Aerodynamic and propulsion forces in body axes */
        double fmb[3];          /* *io (N*m)    Aerodynamic and propulsion moment in body axes */
        double fap[3];          /* *io (N)      Aerodynamic force in body axes */
/**************************************************************************************/
};


#endif // __forces_HH__
