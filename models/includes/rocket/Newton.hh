#ifndef __newton_HH__
#define __newton_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the newton Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Newton.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/

#include <armadillo>

#include "aux/global_constants.hh"
#include "aux/aux.hh"

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
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & kinematics;
            ar & environment;
            ar & euler;
            ar & propulsion;
            ar & forces;

            ar & alt;
            ar & lonx;
            ar & latx;
            ar & _SBII;
            ar & _VBII;
            ar & _ABII;

            ar & _TGI;
            ar & _TDI;
            ar & aero_loss;
            ar & gravity_loss;

            ar & _FSPB;
        }

        Newton(Kinematics &kine, _Euler_ &elr, Environment &env, Propulsion &prop, Forces &forc);
        Newton(const Newton& other);

        Newton& operator= (const Newton& other);

        void initialize();
        void propagate(double int_step);
        void update_diagnostic_attributes(double int_step);

        void load_location(double lonx, double latx, double alt);
        void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);

        double get_alt();
        double get_lonx();
        double get_latx();
        double get_dvbe();
        double get_dbi();
        double get_dvbi();
        double get_thtvdx();
        double get_psivdx();

        arma::vec3 get_FSPB();

        arma::vec3 get_SBII();
        arma::vec3 get_VBII();
        arma::vec3 get_VBED();
        arma::mat get_TGI();

        unsigned int get_liftoff();

        // XXX: Use getter and setters
        /* Interfacing Variabes */

    private:
        void default_data();

        /* Internal Initializers */
        arma::mat build_WEII();
        arma::vec build_VBEB(double _alpha0x, double _beta0x, double _dvbe);

        /* Internal Propagator / Calculators */
        arma::vec3 calculate_fspb(arma::vec3 FAPB, double vmass);

        void propagate_position_speed_acceleration(double int_step);
        void propagate_aeroloss(double int_step);
        void propagate_gravityloss(double int_step);

        /* Internal Updaters */
        void orbital(arma::vec3 SBII, arma::vec3 VBII, double dbi);

        /* Routing references */
        Kinematics  *kinematics;
        Environment *environment;
        _Euler_     *euler;
        Propulsion  *propulsion;
        Forces      *forces;

        /* Constants */
        arma::mat WEII;     /* *o  (r/s)    Earth's angular velocity (skew-sym) */
        double _WEII[3][3];    /* **  (r/s)    Earth's angular velocity (skew-sym) */

        /* Propagative Stats */
        double alt;           /* *o  (m)      Vehicle altitude */
        double lonx;          /* *o  (d)      Vehicle longitude */
        double latx;          /* *o  (d)      Vehicle latitude */

        arma::vec SBII;      /* *o  (m)      Vehicle position in inertia coord */
        double _SBII[3];       /* *o  (m)      Vehicle position in inertia coord */

        arma::vec VBII;      /* *o  (m/s)    Vehicle inertia velocity */
        double _VBII[3];       /* *o  (m/s)    Vehicle inertia velocity */

        arma::vec ABII;      /* *o  (m/s2)   Vehicle inertia acceleration */
        double _ABII[3];      /* *o  (m/s2)   Vehicle inertia acceleration */

        arma::mat TDI;      /* **  (--)     Transformation Matrix of geodetic wrt inertial  coordinates */
        double _TDI[3][3];     /* **  (--)     Transformation Matrix of geodetic wrt inertial  coordinates */

        arma::mat TGI;      /* **  (--)     Transformation Matrix geocentric wrt inertia coord */
        double _TGI[3][3];     /* **  (--)     Transformation Matrix geocentric wrt inertia coord */

        double aero_loss;     /* **  (m/s)    Velocity loss caused by aerodynamic drag */
        double gravity_loss;  /* **  (m/s)    Velocity loss caused by gravity */

        /* Generating Outputs */
        arma::vec FSPB;       /* *o  (m/s2)   Specific force in body coord */
        double _FSPB[3];       /* *o  (m/s2)   Specific force in body coord */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
        double _TVD[3][3];     /* **  (--)    Transformation Matrix of geographic velocity wrt geodetic coord */
        double _VBED[3];       /* *o  (m/s)   [DIAG] Geographic velocity in geodetic coord */
        double _grndtrck;      /* *o  (m)     [DIAG] Vehicle ground track on earth */
        double _gndtrkmx;      /* *o  (km)    [DIAG] Ground track - km */
        double _gndtrnmx;      /* **  (nm)    [DIAG] Ground track - nm */
        double _ayx;           /* *o  (m/s2)  [DIAG] Achieved side acceleration */
        double _anx;           /* *o  (m/s2)  [DIAG] Achieved normal acceleration */
        double _dbi;           /* *o  (m)     [DIAG] Vehicle distance from center of earth */
        double _dvbi;          /* *o  (m/s)   [DIAG] Vehicle inertia speed */
        double _dvbe;          /* *o  (m/s)   [DIAG] Vehicle geographic speed */
        double _thtvdx;        /* *o  (d)     [DIAG] Vehicle's flight path angle */
        double _psivdx;        /* *o  (d)     [DIAG] Vehicle's heading angle */
        unsigned int liftoff;        /* *i  (--)     To check wether the rocket liftoff or not: liftoff = 1, not liftoff = 0 */

        /* Orbital Logging */
        double _inclination;   /* *o  (deg)   [DIAG] Orbital inclination is the minimun angle between reference plane and the orbital plane or direction of an object in orbit around another object */
        double _eccentricity;  /* *o  (--)    [DIAG] Determines the amount by which its orbit around another body deviates from a perfect circle */
        double _semi_major;    /* *o  (m)     [DIAG] the major axis of an ellipse is its longest diameter */
        double _ha;            /* *o  (m)     [DIAG] Orbital Apogee */
        double _hp;            /* *o  (m)     [DIAG] Orbital Perigee */
        double _lon_anodex;    /* *o  (deg)   [DIAG] The longitude of the ascending node (☊ or Ω) is one of the orbital elements used to specify the orbit of an object in space. It is the angle from a reference direction, called the origin of longitude, to the direction of the ascending node, measured in a reference plane */
        double _arg_perix;     /* *o  (deg)   [DIAG] The argument of periapsis (also called argument of perifocus or argument of pericenter), symbolized as ω, is one of the orbital elements of an orbiting body. Parametrically, ω is the angle from the body's ascending node to its periapsis, measured in the direction of motion */
        double _true_anomx;    /* *o  (deg)   [DIAG] In celestial mechanics, true anomaly is an angular parameter that defines the position of a body moving along a Keplerian orbit. It is the angle between the direction of periapsis and the current position of the body, as seen from the main focus of the ellipse (the point around which the object orbits) */
        double _ref_alt;       /* *o  (m)     [DIAG] */
};

#endif  // __newton_HH__
