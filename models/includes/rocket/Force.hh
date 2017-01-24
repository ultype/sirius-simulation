#ifndef __forces_HH__
#define __forces_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the forces Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Forces.cpp))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include "aux/global_constants.hh"
#include "aux/utility_header.hh"
#include "Environment.hh"
#include "Rcs.hh"
#include "Tvc.hh"
#include "Kinematics.hh"
#include "Newton.hh"
#include "Control.hh"
#include "Propulsion.hh"
class Propulsion;
class AeroDynamics;
class RCS;
class TVC;
class Environment;

class Forces{
    public:
        Forces(){};
         void init_force(Environment* env, Propulsion* prop, RCS* rcs
             , AeroDynamics* aero, TVC* tvc);
         void forces();

         Environment *environment;
         Propulsion *propulsion;
         RCS *rcs;
         AeroDynamics *Aerodynamics;
         TVC *tvc;

         Matrix get_FAPB();
         double* get_fapb_ptr();

         Matrix get_FAP();
         double* get_fap_ptr();

         Matrix get_FMB();

    private:
        double fapb[3];         /* *io (N)      Aerodynamic and propulsion forces in body axes */
        double fap[3];          /* *io (N)      Aerodynamic force in body axes */
        double fmb[3];          /* *io (N*m)    Aerodynamic and propulsion moment in body axes */

};


#endif // __forces_HH__
