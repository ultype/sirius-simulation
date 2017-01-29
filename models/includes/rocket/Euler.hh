#ifndef __euler_HH__
#define __euler_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the forces Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Euler.cpp))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include <armadillo>

#include "aux/global_constants.hh"
#include "aux/utility_header.hh"
#include "Environment.hh"
#include "Kinematics.hh"
#include "Newton.hh"
#include "Force.hh"
#include "Propulsion.hh"

class Propulsion;
class Kinematics;
class Forces;

class _Euler_ {
    TRICK_INTERFACE(_Euler_);

    public:
        _Euler_(Kinematics& kine, Propulsion& prop, Forces& forc);
        _Euler_(const _Euler_& other);

        _Euler_& operator=(const _Euler_& other);

        void initialize();

        void propagate(double int_step);
        void update_diagnostic_attributes(double int_step);

        double get_ppx();
        double get_qqx();
        double get_rrx();
        Matrix get_WBII();
        Matrix get_WBIB();
        arma::vec3 get_WBII_();
        arma::vec3 get_WBIB_();

        void load_angular_velocity(double ppx, double qqx, double rrx);

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */
        void propagate_WBIB(double int_step, arma::vec3 FMB, arma::mat33 IBBB);

        /* Internal Calculators */
        arma::vec3 calculate_WBII(arma::mat33 TBI);

        arma::vec3 calculate_WBEB(arma::mat33 TBI);

        /* Routing references */
        Kinematics * kinematics;
        Propulsion * propulsion;
        Forces     * forces;

        /* Constants */
        arma::vec WEII; /* ** */
        double _WEII[3]; /* ** */

        /* Propagative Stats */

        arma::vec WBIB;   /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */
        double _WBIB[3];  /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */
        arma::vec WBIBD;  /* *io (r/s2)       Angular velocity of vehicle wrt inertia in body coord - derivative */
        double _WBIBD[3]; /* *io (r/s2)       Angular velocity of vehicle wrt inertia in body coord - derivative */

        /* Generating Outputs */
        arma::vec WBII;   /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */
        double _WBII[3];  /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */

        arma::vec WBEB;   /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */
        double _WBEB[3];  /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */

        double ppx;     /* *io (d/s)        Body roll angular velocity wrt earth in body axes */
        double qqx;     /* *io (d/s)        Body pitch angular velocity wrt earth in body axes */
        double rrx;     /* *io (d/s)        Body yaw angular velocity wrt earth in body axes */
};


#endif // __euler_HH__
