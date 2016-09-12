#ifndef __newton_HH__
#define __newton_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the newton Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/newton.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/

#include "global_constants.hh"
#include "utility_header.hh"

class Newton {
    public:
    Newton() {}

    Matrix VBEB(3,1);
    Matrix TVD(3,3);
    Matrix WBII(3,1);
    Matrix VBED(3,1);
    Matrix SBII(3,1);
    Matrix VBII(3,1);
    Matrix ABII(3,1);
    Matrix WEII(3,3);
    Matrix TDI(3,3);
    Matrix TGI(3,3);
    Matrix FSPB(3,1);

    void default_newton();
    void initialize_newton(kinematics* kine);
    void newton(double int_step, Kinematics *kine, Environment *Env, Propulsion *Prop, forces *forc);
/***********************************************Variables description******************************************/
    double alpha0x;     /* *io (d)      Initial angle-of-attack */
    double beta0x;      /* *io (d)      Initial sideslip-angle */
    double lonx;        /* *io (d)      Vehicle longitude */
    double latx;        /* *io (d)      Vehicle latitude */
    double alt;         /* *io (m)      Vehicle altitude */
    double tvd[9];      /* *io (--)     Transformation Matrix of geographic velocity wrt geodetic coord */
    double tdi[9];      /* *io (--)     Transformation Matrix of geodetic wrt inertial  coordinates */
    double dvbe;        /* *io (m/s)    Vehicle geographic speed */
    double dvbi;        /* *io (m/s)    Vehicle inertia speee */
    double weii[9];     /* *io (r/s)    Earth's angular velocity (skew-sym) */
    double psivdx;      /* *io (d)      Vehicle's heading angle */
    double thtvdx;      /* *io (d)      Vehicle's flight path angle */
    double dbi;         /* *io (m)      Vehicle distance from center of earth */
    double tgi[9];      /* *io (--)     Transformation Matrix geocentric wrt inertia coord */
    double vbed[3];     /* *io (m/s)    Geographic velocity in geodetic coord */
    double altx;        /* *io (ft)     Vehicle altitude - ft */
    double IPos[3];     /* *io (m)      Vehicle position in inertia coord */
    double IVel[3];     /* *io (m/s)    Vehicle inertia velocity */
    double IAccl[3];    /* *io (m/s2)  Vehicle inertia acceleration */
    double grndtrck;    /* *io (m)      Vehicle ground track on earth */
    double fspb[3];     /* *io (m/s2)  Specific force in body coord */
    double ayx;         /* *io (m/s2)  Achieved side acceleration */
    double anx;         /* *io (m/s2)  Achieved normal acceleration */
    double gndtrkmx;    /* *io (km)     Ground track - km */
    double gndtrnmx;    /* *io (nm)     Ground track - nm */
    double aero_loss;   /* *io (m/s)    Speed loss caused by aerodynamic drag */
    double gravity_loss;/* *io (m/s)    Speed loss caused by gravity */
/*************************************************************************************************************/



};

#endif  // __newton_HH__
