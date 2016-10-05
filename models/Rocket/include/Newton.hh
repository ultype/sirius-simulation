#ifndef __newton_HH__
#define __newton_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the newton Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Newton.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/

#include "global_constants.hh"
#include "utility_header.hh"

#include "Environment.hh"
#include "Kinematics.hh"
#include "Propulsion.hh"
#include "Force.hh"
#include "Euler.hh"

class Kinematics;

class Propulsion;

class Newton {
    public:
    Newton() {}

    Kinematics *kinematics;
    Environment *environment;
    _Euler_ *euler;
    Propulsion *propulsion;
    Forces *forces;

    void default_data();
    void initialize(Kinematics *kine, _Euler_ *elr, Environment *Env, Propulsion *Prop, Forces *forc);
    void calculate_newton(double int_step);
    void orbital(Matrix &SBII, Matrix &VBII, double &dbi);
    /* Input data */
/***********************************************Variables description******************************************/
    double alpha0x;     /* *i (d)      Initial angle-of-attack */
    double beta0x;      /* *i (d)      Initial sideslip-angle */

    double dvbe;        /* *io (m/s)    Vehicle geographic speed */
    double lonx;        /* *io (d)      Vehicle longitude */
    double latx;        /* *io (d)      Vehicle latitude */
    double alt;         /* *io (m)      Vehicle altitude */

    double psivdx;      /* *io (d)      Vehicle's heading angle */
    double thtvdx;      /* *io (d)      Vehicle's flight path angle */

    double tvd[3][3];   /* *io (--)     Transformation Matrix of geographic velocity wrt geodetic coord */
    double tdi[3][3];   /* *io (--)     Transformation Matrix of geodetic wrt inertial  coordinates */

    double dvbi;        /* *io (m/s)    Vehicle inertia speee */
    double weii[3][3];  /* *io (r/s)    Earth's angular velocity (skew-sym) */
    double dbi;         /* *io (m)      Vehicle distance from center of earth */
    double tgi[3][3];   /* *io (--)     Transformation Matrix geocentric wrt inertia coord */
    double vbed[3];     /* *io (m/s)    Geographic velocity in geodetic coord */
    double altx;        /* *io (ft)     Vehicle altitude - ft */
    double IPos[3];     /* *io (m)      Vehicle position in inertia coord */
    double IVel[3];     /* *io (m/s)    Vehicle inertia velocity */
    double IAccl[3];    /* *io (m/s2)   Vehicle inertia acceleration */
    double grndtrck;    /* *io (m)      Vehicle ground track on earth */
    double fspb[3];     /* *io (m/s2)   Specific force in body coord */
    double ayx;         /* *io (m/s2)   Achieved side acceleration */
    double anx;         /* *io (m/s2)   Achieved normal acceleration */
    double gndtrkmx;    /* *io (km)     Ground track - km */
    double gndtrnmx;    /* *io (nm)     Ground track - nm */
    double aero_loss;   /* *io (m/s)    Velocity loss caused by aerodynamic drag */
    double gravity_loss;/* *io (m/s)    Velocity loss caused by gravity */
    double gravity_loss2;/* *io (m/s)   Velocity loss caused by gravity */
    double inclination; /* *io (deg)    Orbital inclination is the minimun angle between reference plane and the orbital plane or direction of an object in orbit around another object*/
    double eccentricity;/* *io (--)     Determines the amount by which its orbit around another body deviates from a perfect circle*/
    double semi_major; /* *io (m)      the major axis of an ellipse is its longest diameter*/
    double ha;          /* *io (m)      Orbital Apogee*/
    double hp;          /* *io (m)      Orbital Perigee*/
    double lon_anodex;  /* *o (deg)     The longitude of the ascending node (☊ or Ω) is one of the orbital elements used to specify the orbit of an object in space. It is the angle from a reference direction, called the origin of longitude, to the direction of the ascending node, measured in a reference plane*/
    double arg_perix;   /* *o (deg)     The argument of periapsis (also called argument of perifocus or argument of pericenter), symbolized as ω, is one of the orbital elements of an orbiting body. Parametrically, ω is the angle from the body's ascending node to its periapsis, measured in the direction of motion*/
    double true_anomx;  /* *o (deg)     In celestial mechanics, true anomaly is an angular parameter that defines the position of a body moving along a Keplerian orbit. It is the angle between the direction of periapsis and the current position of the body, as seen from the main focus of the ellipse (the point around which the object orbits)*/
    double ref_alt;     /* *io (m)      */
/*************************************************************************************************************/



};

#endif  // __newton_HH__
