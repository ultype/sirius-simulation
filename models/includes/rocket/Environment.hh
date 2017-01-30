#ifndef __environment_HH__
#define __environment_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Environment Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Environment.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "aux/global_constants.hh"
#include "aux/utility_header.hh"
#include "Newton.hh"
#include "Kinematics.hh"
#include "Aerodynamics.hh"

#include "cad/env/atmosphere.hh"
#include "cad/env/wind.hh"

class Environment{
    TRICK_INTERFACE(Environment);

    public:
        Environment(Newton &newt, AeroDynamics &aero, Kinematics &kine);
        Environment(const Environment& other);
        ~Environment();

        Environment& operator=(const Environment& other);

        void atmosphere_use_public();
        void atmosphere_use_nasa();
        void atmosphere_use_weather_deck(char* filename);

        void set_no_wind();
        void set_constant_wind(double dvae, double dir, double twind, double vertical_wind);
        void set_tabular_wind(char* filename, double twind, double vertical_wind);

        void set_no_wind_turbulunce();
        void set_wind_turbulunce();

        void initialize();
        void calculate_env(double int_step);
        arma::vec3 environment_dryden(double dvba,double int_step);
///////////////////////////////////////////////////////////////////////////////
//Definition of environment module-variables
//Member function of class 'DM'
//Ref: Zipfel, Section 10.3.2, p. 465
//Module-variable locations are assigned to DM[50-99]
//
// The switch 'mair' controls the atmosphere, wind and air turbulence options:
//
//     mair=|matmo|mturb|mwind|
//
//           matmo = 0 US 1976 Standard Atmosphere (public domain shareware)
//                 = 1 US 1976 Standard Atmosphere with extension up to 1000 km (NASA Marshall)
//                 = 2 tabular atmosphere from WEATHER_DECK
//
//                 mturb = 0 no turbulence
//                       = 1 dryden turbulence model
//
//                       mwind =
//                             = 1 constant wind, input: dvaeg,psiwdx
//                             = 2
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
/***********************************Variables describtion******************************/

        double get_rho();
        double get_vmach();
        double get_pdynmc();
        double get_tempk();
        double get_dvba();
        double get_grav();
        double get_press();

        Matrix get_GRAVG();
        Matrix get_VAED();
        arma::vec3 get_GRAVG_();
        arma::vec3 get_VAED_();

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */

        /* Internal Calculators */

        /* Routing references */
        Newton       * newton;
        Kinematics   * kinematics;
        AeroDynamics * aerodynamics;

        /* Constants */
        enum {
            NO_TURBULENCE = 0,
            DRYDEN_TURBULENCE
        } ENV_WIND_TURB_TYPE;

        int mturb;          /* *o  (--)         see ENV WIND TURB TYPE */

        cad::Atmosphere * atmosphere;
        cad::Wind       * wind;

        /* Propagative Stats */

        /* Generating Outputs */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */

        arma::vec GRAVG;    /* *io (m/s2)       Gravity acceleration in geocentric coord */
        double _GRAVG[3];   /* *io (m/s2)       Gravity acceleration in geocentric coord */

        double rho;         /* *io (kg/m3)      Atmospheric Density */
        double vmach;       /* *io (--)         Mach number */
        double pdynmc;      /* *io (pa)         Dynamic pressure */

        double tempk;       /* *io (K)          Atmospheric temperature */
        double dvba;        /* *io (m/s)        Vehicle speed wrt air */
        //double gravg[3];    [> *io (m/s2)       Gravity acceleration in geocentric coord <]
        //double vaed[3];     [> *io (m/s)        Smoothed wind velocity in geodetic coord <]
        double grav;        /* *io (m/s2)       Magnitude of gravity acceleration */
        double press;       /* *io (pa)         Atmospheric pressure */


        double vsound;      /* *io (m/s)        Sonic speed */
        double dvae;        /* *io (m/s)        Madnitude of constant air speed */
        double turb_length; /* *io (m)          Turbulence correlation length - m*/
        double turb_sigma;  /* *io (m/s)        Turbulence magnitude (1sigma) - m/s*/
        double taux1;       /* *io (--)         First turbulence state variable - ND*/
        double taux1d;      /* *io (--)         First turbulence state variable - ND*/
        double taux2;       /* *io (1/s)        First turbulence state variable derivative - 1/s*/
        double taux2d;      /* *io (1/s)        First turbulence state variable derivative - 1/s*/
        double tau;         /* *io (m/s)        Turblence velocity component in load factor plane - m/s*/
        double gauss_value; /* *io (--)         White Gaussian noise - ND*/
        double tempc;       /* *io (c)          Atmospheric temperature - Centigrade*/
        //double vaeds[3];    [> *io (m/s)        Smoothed wind velocity in geodetic coord - m/s<]
        //double vaedsd[3];   [> *io (m/s)        Smoothed wind velocity derivative - m/s<]
        double markov_value;/* *io (m/s)        Markov variable - m/s*/
        double vaed3;       /* *io (m/s)        Vertical air speed (pos.down) - m/s*/
        double psiwdx;      /* *io (m/s)        Wind direction from north - m/s*/
        double twind;       /* *io (s)          Wind smoothing time constant - sec*/
};

#endif  // __environment_HH__
