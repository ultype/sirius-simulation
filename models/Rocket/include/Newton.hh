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
#include "aux.hh"

#include "Environment.hh"
#include "Kinematics.hh"
#include "Propulsion.hh"
#include "Force.hh"
#include "Euler.hh"

class Kinematics;

class Propulsion;

class Newton {
    TRICK_INTERFACE(Newton);

    public:
        Newton(Kinematics &kine, _Euler_ &elr, Environment &env, Propulsion &prop, Forces &forc);

        void default_data();
        void initialize();
        void calculate_newton(double int_step);
        void orbital(Matrix &SBII, Matrix &VBII, double &dbi);

        double get_alt() { return alt; };
        double get_dvbe() { return dvbe; };
        double get_dbi() { return dbi; };
        double get_dvbi() { return dvbi; };
        double get_thtvdx() { return thtvdx; };
        double get_lonx() { return lonx; };
        double get_latx() { return latx; };
        Matrix get_IPos() { return Matrix(IPos); };
        Matrix get_IVel() { return Matrix(IVel); };
        Matrix get_FSPB() { return Matrix(fspb); };
        Matrix get_VBED() { return Matrix(vbed); };

        // XXX: Use getter and setters
        /* Interfacing Variabes */
        double alpha0x;     /* *i (d)      Initial angle-of-attack */
        double beta0x;      /* *i (d)      Initial sideslip-angle */
        double alt;         /* *io (m)      Vehicle altitude */
        double lonx;        /* *io (d)      Vehicle longitude */
        double latx;        /* *io (d)      Vehicle latitude */
        double dvbe;        /* *io (m/s)    Vehicle geographic speed */
        double dbi;         /* *o  (m)      Vehicle distance from center of earth */
        double dvbi;        /* *o  (m/s)    Vehicle inertia speee */
        double thtvdx;      /* *o  (d)      Vehicle's flight path angle */
        double fspb[3];     /* *o  (m/s2)   Specific force in body coord */
        double vbed[3];     /* *o  (m/s)    Geographic velocity in geodetic coord */
        double IPos[3];     /* *o  (m)      Vehicle position in inertia coord */
        double IVel[3];     /* *o  (m/s)    Vehicle inertia velocity */

    private:
        Matrix build_WEII();
        Matrix build_VBEB(double _alpha0x, double _beta0x, double _dvbe);

        /* Routing references */
        Kinematics  *kinematics;
        Environment *environment;
        _Euler_     *euler;
        Propulsion  *propulsion;
        Forces      *forces;

        /* Internal Variables */
/***********************************************Variables description******************************************/
        double psivdx;        /* *o (d)      Vehicle's heading angle */
        double tvd[3][3];     /* ** (--)     Transformation Matrix of geographic velocity wrt geodetic coord */
        double tdi[3][3];     /* ** (--)     Transformation Matrix of geodetic wrt inertial  coordinates */
        double weii[3][3];    /* ** (r/s)    Earth's angular velocity (skew-sym) */
        double tgi[3][3];     /* ** (--)     Transformation Matrix geocentric wrt inertia coord */
        double altx;          /* ** (ft)     Vehicle altitude - ft */
        double IAccl[3];      /* ** (m/s2)   Vehicle inertia acceleration */
        double grndtrck;      /* ** (m)      Vehicle ground track on earth */
        double ayx;           /* ** (m/s2)   Achieved side acceleration */
        double anx;           /* ** (m/s2)   Achieved normal acceleration */
        double gndtrkmx;      /* ** (km)     Ground track - km */
        double gndtrnmx;      /* ** (nm)     Ground track - nm */
        double aero_loss;     /* ** (m/s)    Velocity loss caused by aerodynamic drag */
        double gravity_loss;  /* ** (m/s)    Velocity loss caused by gravity */
        double gravity_loss2; /* ** (m/s)    Velocity loss caused by gravity */
        double inclination;   /* ** (deg)    Orbital inclination is the minimun angle between reference plane and the orbital plane or direction of an object in orbit around another object*/
        double eccentricity;  /* ** (--)     Determines the amount by which its orbit around another body deviates from a perfect circle*/
        double semi_major;    /* ** (m)      the major axis of an ellipse is its longest diameter*/
        double ha;            /* ** (m)      Orbital Apogee*/
        double hp;            /* ** (m)      Orbital Perigee*/
        double lon_anodex;    /* ** (deg)    The longitude of the ascending node (☊ or Ω) is one of the orbital elements used to specify the orbit of an object in space. It is the angle from a reference direction, called the origin of longitude, to the direction of the ascending node, measured in a reference plane*/
        double arg_perix;     /* ** (deg)    The argument of periapsis (also called argument of perifocus or argument of pericenter), symbolized as ω, is one of the orbital elements of an orbiting body. Parametrically, ω is the angle from the body's ascending node to its periapsis, measured in the direction of motion*/
        double true_anomx;    /* ** (deg)    In celestial mechanics, true anomaly is an angular parameter that defines the position of a body moving along a Keplerian orbit. It is the angle between the direction of periapsis and the current position of the body, as seen from the main focus of the ellipse (the point around which the object orbits)*/
        double ref_alt;       /* ** (m)      */
/*************************************************************************************************************/
};

#endif  // __newton_HH__
