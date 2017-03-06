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
#include "cad/env/atmosphere76.hh"
#include "cad/env/atmosphere_nasa2002.hh"
#include "cad/env/atmosphere_weatherdeck.hh"

#include "cad/env/wind.hh"
#include "cad/env/wind_no.hh"
#include "cad/env/wind_tabular.hh"
#include "cad/env/wind_constant.hh"

class Environment{
    TRICK_INTERFACE(Environment);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar.template register_type<cad::Atmosphere76>();
            ar.template register_type<cad::Atmosphere_nasa2002>();
            ar.template register_type<cad::Atmosphere_weatherdeck>();

            ar.template register_type<cad::Wind_No>();
            ar.template register_type<cad::Wind_Constant>();
            ar.template register_type<cad::Wind_Tabular>();

            ar & atmosphere;
            ar & wind;

            ar & kinematics  ;
            ar & newton      ;
            ar & aerodynamics;

            ar & _GRAVG;
            ar & vmach;
            ar & pdynmc;
            ar & dvba;
        }

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
        void set_wind_turbulunce(double turb_length, double turb_sigma,
                                    double taux1, double taux1d,
                                    double taux2, double taux2d,
                                    double tau,   double gauss_value);

        void initialize();
        void propagate(double int_step);
        void update_diagnostic_attributes(double int_step);

        double get_rho();
        double get_vmach();
        double get_pdynmc();
        double get_tempk();
        double get_dvba();
        double get_grav();
        double get_press();

        arma::vec3 get_GRAVG();
        arma::vec3 get_VAED();

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
        cad::Atmosphere * atmosphere;
        cad::Wind       * wind;

        /* Propagative Stats */

        /* Generating Outputs */
        arma::vec GRAVG;    /* *o (m/s2)       Gravity acceleration in geocentric coord */
        double _GRAVG[3];   /* *o (m/s2)       Gravity acceleration in geocentric coord */

        double vmach;       /* *o (--)         Mach number */
        double pdynmc;      /* *o (pa)         Dynamic pressure */
        double dvba;        /* *o (m/s)        Vehicle speed wrt air */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
        double gravg;       /* *o (m/s2)       Magnitude of gravity acceleration */
        double tempc;       /* *o (c)          Atmospheric temperature - Centigrade*/
};

#endif  // __environment_HH__
