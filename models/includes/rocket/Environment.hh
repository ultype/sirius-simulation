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
#include "Time_management.hh"
#include "rocket/dm_delta_ut.hh"

class time_management;

class Environment{
    TRICK_INTERFACE(Environment);

    public:
       Environment(Newton &newt, AeroDynamics &aero, Kinematics &kine, time_management &Time);
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
        void set_wind_turbulunce(double turb_length, double turb_sigma,
                                    double taux1, double taux1d,
                                    double taux2, double taux2d,
                                    double tau,   double gauss_value);

        void initialize();
        void propagate(double int_step);
        void update_diagnostic_attributes(double int_step);
        void dm_RNP();

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
        time_management *time;

        /* Constants */
        cad::Atmosphere * atmosphere;
        cad::Wind       * wind;

        /* Propagative Stats */

        /* Generating Outputs */
        arma::vec GRAVG;    /* *o (m/s2)       Gravity acceleration in geocentric coord */
        arma::mat TEI;    /* *io  (--)         Transformation matrix for ECI to ECEF */
        double _GRAVG[3];   /* *o (m/s2)       Gravity acceleration in geocentric coord */
        double _TEI[3][3];  /* *io  (--)       Transformation matrix for ECI to ECEF */
        double vmach;       /* *o (--)         Mach number */
        double pdynmc;      /* *o (pa)         Dynamic pressure */
        double dvba;        /* *o (m/s)        Vehicle speed wrt air */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
        double gravg;       /* *o (m/s2)       Magnitude of gravity acceleration */
        double tempc;       /* *o (c)          Atmospheric temperature - Centigrade*/
        double DM_sidereal_time; /* *io  (r)  temps_sideral */
        double DM_Julian_century; /* *io  (--)  Julian_century */
        double DM_w_precessing; /* *io  (--)  w_precessing */
};

#endif  // __environment_HH__
