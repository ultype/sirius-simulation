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
        _Euler_(){};
        void initialization(Kinematics* kine, Propulsion* prop, Forces* forc);
        void euler(double int_step);

        Kinematics *kinematics;
        Propulsion *propulsion;
        Forces *force;
/***********************************Variables describtion******************************/
        double ppx;     /* *io (d/s)        Body roll angular velocity wrt earth in body axes */
        double qqx;     /* *io (d/s)        Body pitch angular velocity wrt earth in body axes */
        double rrx;     /* *io (d/s)        Body yaw angular velocity wrt earth in body axes */
        double wbii[3]; /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */
        double wbib[3]; /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */
    private:
        double wbeb[3]; /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */
        double wbibd[3];/* *io (r/s2)      Angular velocity of vehicle wrt inertia in body coord - derivative */
/**************************************************************************************/
};


#endif // __euler_HH__
