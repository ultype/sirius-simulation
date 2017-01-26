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
    public:
        _Euler_(Kinematics& kine, Propulsion& prop, Forces& forc);
        _Euler_(const _Euler_& other);

        _Euler_& operator=(const _Euler_& other);

        void initialize();
        void euler(double int_step);

        Kinematics *kinematics;
        Propulsion *propulsion;
        Forces *forces;
/***********************************Variables describtion******************************/

        double get_ppx();
        double get_qqx();
        double get_rrx();
        Matrix get_WBII();
        Matrix get_WBIB();

        void load_angular_velocity(double ppx, double qqx, double rrx);

    private:
        void default_data();

        double weii[3]; /* *io (r/s)         */

        double ppx;     /* *io (d/s)        Body roll angular velocity wrt earth in body axes */
        double qqx;     /* *io (d/s)        Body pitch angular velocity wrt earth in body axes */
        double rrx;     /* *io (d/s)        Body yaw angular velocity wrt earth in body axes */
        double wbii[3]; /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */
        double wbib[3]; /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */

        double wbeb[3]; /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */
        double wbibd[3];/* *io (r/s2)      Angular velocity of vehicle wrt inertia in body coord - derivative */
/**************************************************************************************/
};


#endif // __euler_HH__
