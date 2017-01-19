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

        void load_location(double lonx, double latx, double alt);
        void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);

        double get_alt();
        double get_lonx();
        double get_latx();
        double get_dvbe();
        double get_dbi();
        double get_dvbi();
        double get_thtvdx();
        Matrix get_IPos();
        Matrix get_IVel();
        Matrix get_FSPB();
        Matrix get_VBED();

        // XXX: Use getter and setters
        /* Interfacing Variabes */

    private:
        /* Internal Initializers */
        Matrix build_WEII();
        Matrix build_VBEB(double _alpha0x, double _beta0x, double _dvbe);

        /* Routing references */
        Kinematics  *kinematics;
        Environment *environment;
        _Euler_     *euler;
        Propulsion  *propulsion;
        Forces      *forces;

        /* Module Outputs */
        double fspb[3];       /* *o  (m/s2)   Specific force in body coord */

        double alt;           /* *o  (m)      Vehicle altitude */
        double lonx;          /* *o  (d)      Vehicle longitude */
        double latx;          /* *o  (d)      Vehicle latitude */

        double dbi;           /* *o  (m)      Vehicle distance from center of earth */
        double dvbi;          /* *o  (m/s)    Vehicle inertia speee */

        double vbed[3];       /* *o  (m/s)    Geographic velocity in geodetic coord */
        double dvbe;          /* *o  (m/s)    Vehicle geographic speed */
        double thtvdx;        /* *o  (d)      Vehicle's flight path angle */
        double psivdx;        /* *o  (d)      Vehicle's heading angle */

        /* Propagative Internal Stats */
        double IPos[3];       /* *o  (m)      Vehicle position in inertia coord */
        double IVel[3];       /* *o  (m/s)    Vehicle inertia velocity */
        double weii[3][3];    /* **  (r/s)    Earth's angular velocity (skew-sym) */
        double tdi[3][3];     /* **  (--)     Transformation Matrix of geodetic wrt inertial  coordinates */
        double tgi[3][3];     /* **  (--)     Transformation Matrix geocentric wrt inertia coord */
        double tvd[3][3];     /* **  (--)     Transformation Matrix of geographic velocity wrt geodetic coord */
        double aero_loss;     /* **  (m/s)    Velocity loss caused by aerodynamic drag */
        double gravity_loss;  /* **  (m/s)    Velocity loss caused by gravity */

        /* Non-propagating Diagnostic Variables */
        double IAccl[3];      /* ** (m/s2)   Vehicle inertia acceleration */
        double grndtrck;      /* ** (m)      Vehicle ground track on earth */
        double gndtrkmx;      /* ** (km)     Ground track - km */
        double gndtrnmx;      /* ** (nm)     Ground track - nm */
        double ayx;           /* ** (m/s2)   Achieved side acceleration */
        double anx;           /* ** (m/s2)   Achieved normal acceleration */

        double inclination;   /* ** (deg)    Orbital inclination is the minimun angle between reference plane and the orbital plane or direction of an object in orbit around another object*/
        double eccentricity;  /* ** (--)     Determines the amount by which its orbit around another body deviates from a perfect circle*/
        double semi_major;    /* ** (m)      the major axis of an ellipse is its longest diameter*/
        double ha;            /* ** (m)      Orbital Apogee*/
        double hp;            /* ** (m)      Orbital Perigee*/
        double lon_anodex;    /* ** (deg)    The longitude of the ascending node (☊ or Ω) is one of the orbital elements used to specify the orbit of an object in space. It is the angle from a reference direction, called the origin of longitude, to the direction of the ascending node, measured in a reference plane*/
        double arg_perix;     /* ** (deg)    The argument of periapsis (also called argument of perifocus or argument of pericenter), symbolized as ω, is one of the orbital elements of an orbiting body. Parametrically, ω is the angle from the body's ascending node to its periapsis, measured in the direction of motion*/
        double true_anomx;    /* ** (deg)    In celestial mechanics, true anomaly is an angular parameter that defines the position of a body moving along a Keplerian orbit. It is the angle between the direction of periapsis and the current position of the body, as seen from the main focus of the ellipse (the point around which the object orbits)*/
        double ref_alt;       /* ** (m)      */
};

#endif  // __newton_HH__
