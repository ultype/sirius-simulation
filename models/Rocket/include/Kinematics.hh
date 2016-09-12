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

class kinematics{
    public:
        Kinematics(){};

        void init_kinematics(double time, double int_step, Newton &newt);

        Matrix TBD(3,3);
        Matrix TBI(3,3);
        Matrix TBID(3,3);
/***********************************Variables describtion******************************/
        //double time;        /* *io (s)     simulation time */
        double tbd[9];      /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
        double tbi[9];      /* *io (--)    Transformation Matrix of body coord wrt inertia coord */
        double tbid[9];     /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */
        double ortho_error; /* *io (--)    Direction cosine matrix orthogonality error*/
        double psibd;       /* *io (r)     Yaw angle of Vehicle wrt geodetic coord */
        double thtbd;       /* *io (r)     Pitch angle of Vehicle wrt geodetic coord */
        double phibd;       /* *io (r)     Roll angle of Vehicle wrt geodetic coord */
        double psibdx;      /* *io (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
        double thtbdx;      /* *io (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
        dobule phibdx;      /* *io (d)     Roll angle of Vehicle wrt geodetic coord - deg */
        double alppx;       /* *io (d)     Total angle of attack */
        double phipx;       /* *io (d)     Aerodynamic roll angle*/
        double alphax;      /* *io (d)     Angle of attack */
        double betax;       /* *io (d)     Sideslip angle */
        double alphaix;     /* *io (d)     Angle of attack, inertia velocity*/
        double betaix;      /* *io (d)     Sideslip angle, inertia velocity*/
/**************************************************************************************/
};


#endif  // __kinematics_HH__
