#ifndef __kinematics_HH__
#define __kinematics_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the kinematics Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Kinematics.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "global_constants.hh"
#include "utility_header.hh"

#include "Newton.hh"
#include "Euler.hh"

class Newton;
class Environment;
class _Euler_;

class Kinematics{
    public:
        Newton *newton;
        Environment *environment;
        _Euler_* euler;

        Kinematics(){};

        void initialize(Newton* newt, Environment* env, _Euler_* eul);
        void calculate_kinematics(double int_step);

/***********************************Variables describtion******************************/
        //double time;        /* *io (s)     simulation time */
        double tbd[3][3];      /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
        double tbi[3][3];      /* *io (--)    Transformation Matrix of body coord wrt inertia coord */
        double alppx;       /* *io (d)     Total angle of attack */
        double phipx;       /* *io (d)     Aerodynamic roll angle*/
        double alphax;      /* *io (d)     Angle of attack */
        double betax;       /* *io (d)     Sideslip angle */
        double psibdx;      /* *io (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
        double thtbdx;      /* *io (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
        double phibdx;      /* *io (d)     Roll angle of Vehicle wrt geodetic coord - deg */
    private:
        double tbid[3][3];     /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */
        double ortho_error; /* *io (--)    Direction cosine matrix orthogonality error*/
        double psibd;       /* *io (r)     Yaw angle of Vehicle wrt geodetic coord */
        double thtbd;       /* *io (r)     Pitch angle of Vehicle wrt geodetic coord */
        double phibd;       /* *io (r)     Roll angle of Vehicle wrt geodetic coord */
        double alphaix;     /* *io (d)     Angle of attack, inertia velocity*/
        double betaix;      /* *io (d)     Sideslip angle, inertia velocity*/
/**************************************************************************************/
};


#endif  // __kinematics_HH__
