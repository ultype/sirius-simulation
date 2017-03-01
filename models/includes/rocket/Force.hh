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

class Forces {
    TRICK_INTERFACE(Forces);

    public:
        Forces(){};

        Forces(Environment& env, Propulsion& prop, RCS& rcs, AeroDynamics& aero, TVC& tvc);
        Forces(const Forces& other);

        Forces& operator=(const Forces& other);

        void initialize();

        void collect_forces_and_propagate();

        Matrix get_FAPB();
        arma::vec get_FAPB_();
        double* get_fapb_ptr();

        Matrix get_FAP();
        arma::vec get_FAP_();
        double* get_fap_ptr();

        Matrix get_FMB();
        arma::vec get_FMB_();

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */

        /* Internal Calculators */

        /* Routing references */
        Environment  * environment;
        Propulsion   * propulsion;
        RCS          * rcs;
        AeroDynamics * aerodynamics;
        TVC          * tvc;

        /* Input */

        /* Constants */

        /* Propagative Stats */

        /* Generating Outputs */
        arma::vec FAPB;         /* *o (N)      Aerodynamic and propulsion forces in body axes */
        double _FAPB[3];        /* *o (N)      Aerodynamic and propulsion forces in body axes */

        arma::vec FAP;          /* *o (N)      Aerodynamic force in body axes */
        double _FAP[3];         /* *o (N)      Aerodynamic force in body axes */

        arma::vec FMB;          /* *o (N*m)    Aerodynamic and propulsion moment in body axes */
        double _FMB[3];         /* *o (N*m)    Aerodynamic and propulsion moment in body axes */

};


#endif // __forces_HH__
