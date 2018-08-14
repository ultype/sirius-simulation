#ifndef __propulsion_HH__
#define __propulsion_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the propulsion Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Propulsion.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "global_constants.hh"
#include <functional>
#include "aux.hh"
#include <armadillo>
#include "datadeck.hh"
#include "sim_services/include/simtime.h"

class Propulsion{
    TRICK_INTERFACE(Propulsion);

 public:
    Propulsion();
    Propulsion(const Propulsion& other);

    Propulsion& operator=(const Propulsion& other);

    void initialize();
    void propagate(double int_step);
    void load_proptable(const char* filename);

    enum THRUST_TYPE {
        NO_THRUST = 0,
        INPUT_THRUST = 3,
        LTG_THRUST = 4,
        HOT_STAGE = 5
    };

    enum STAGE {
        STAGE_1 = 0,
        STAGE_2 = 1,
        FARING_SEP = 2,
        STAGE_3 = 3
    };

    void set_stage_2();
    void set_stage_3();
    void set_faring_sep();
    void engine_ignition();
    void set_no_thrust();
    void set_input_thrust(double xcg0, double xcg1,
                            double moi_roll0, double moi_roll1,
                            double moi_pitch0, double moi_pitch1,
                            double moi_yaw0, double moi_yaw1,
                            double spi, double fuel_flow_rate);
    void set_ltg_thrust();
    void get_input_file_var(double xcg0, double xcg1,
                                double moi_roll0, double moi_roll1,
                                double moi_pitch0, double moi_pitch1,
                                double moi_yaw0, double moi_yaw1,
                                double spi, double fuel_flow_rate);
    void set_CG_OFFSET(unsigned int in);
    void set_TWD(unsigned int in);
    void set_ignition_time();

    void set_S2_E1_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10);
    void set_S2_E2_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10);
    void set_S2_E3_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10);
    void set_S2_E4_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10);

    void set_S2_structure_mass(double in);
    void set_S2_propellant_mass(double in);
    void set_S2_spi(double in);
    void set_S3_structure_mass(double in);
    void set_S3_propellant_mass(double in);
    void set_S3_spi(double in);
    void set_faring_mass(double in);
    void set_S2_remaining_fuel_mass(double in);
    void set_S3_remaining_fuel_mass(double in);

    void set_HOT_STAGE();
    // XXX: get_thrust_state
    enum THRUST_TYPE get_thrust_state();
    int get_mprop();
    double get_vmass();
    arma::vec3 get_xcg();
    double get_thrust();
    double get_fmassr();
    arma::vec3 get_xcg_0();
    double get_oxidizer_mass();

    double get_S2_E1_xcg();
    double get_S2_E2_xcg();
    double get_S2_E3_xcg();
    double get_S2_E4_xcg();

    double get_S2_E1_mass();
    double get_S2_E2_mass();
    double get_S2_E3_mass();
    double get_S2_E4_mass();

    arma::mat33 get_I_S2_E1();
    arma::mat33 get_I_S2_E2();
    arma::mat33 get_I_S2_E3();
    arma::mat33 get_I_S2_E4();

    arma::vec3 get_structure_XCG();

    arma::mat33 get_IBBB();

    /* Input File */
    void set_vmass0(double);
    void set_fmass0(double);

    void set_aexit(double);
    void set_payload(double);

    std::function<double()> grab_press;
    std::function<arma::vec3()> grab_SLOSH_CG;
    std::function<double()> grab_slosh_mass;
    std::function<arma::vec3()> grab_e1_XCG;
    std::function<arma::vec3()> grab_e2_XCG;
    std::function<arma::vec3()> grab_e3_XCG;
    std::function<arma::vec3()> grab_e4_XCG;
    std::function<double()> grab_alt;

 private:
    Datadeck proptable;

    /* Internal Getter */
    arma::mat33 get_IBBB0();
    arma::mat33 get_IBBB1();

    /* Internal Initializers */
    void default_data();

    /* Internal Propagator / Calculators */
    void propagate_thrust_delta_v(double int_step, double spi, double fuel_flow_rate, double vmass);
    void propagate_fmasse(double int_step);
    void fuel_expend_integrator(double int_step, unsigned int flag);

    /* Internal Calculators */
    double calculate_thrust(double press);
    double calculate_fmassr();
    void calcuate_engine_variable();

    arma::vec3 calculate_xcg();
    arma::mat33 calculate_IBBB();

    /* Constants */
    arma::vec xcg_0;
    double _xcg_0[3];           /* *o (m)      Initial cg location from nose*/

    arma::vec xcg_1;
    double _xcg_1[3];           /* *o (m)      Final cg location from nose*/

    double moi_roll_0;      /* *o (kg*m2)  Roll MOI of vehicle, initial*/
    double moi_roll_1;      /* *o (kg*m2)  Roll MOI of vehicle, burn-out*/
    double moi_pitch_0;     /* *o (kg*m2)  Pitch MOI of vehicle, initial*/
    double moi_pitch_1;     /* *o (kg*m2)  Pitch MOI of vehicle, burn-out*/
    double moi_yaw_0;     /* *o (kg*m2)  Yaw MOI of vehicle, initial*/
    double moi_yaw_1;     /* *o (kg*m2)  Yaw MOI of vehicle, burn-out*/
    double fuel_flow_rate;  /* *o (kg/s)   Fuel flow rate of rocket motor*/
    double spi;             /* *o (s)      Specific impulse*/

    double aexit;           /* *o (m2)     Nozzle exit area*/
    double payload;         /* *o (kg)     payload mass*/

    double vmass0;          /* *o (kg)     Initial vehicle mass*/
    double fmass0;          /* *o (kg)     Initial fuel mass in stage*/

    /* State */
    enum THRUST_TYPE thrust_state;   /* *o (--)     Propulsion mode, See THRUST TYPE*/
    enum STAGE stage;

    /* Propagative Stats */
    double fmasse;              /* *o (kg)     Fuel mass expended (zero initialization required)*/
    double fmassed;              /* *o (kg/s)   Fuel mass expended derivative*/
    double thrust_delta_v;      /* *o (m/s)    delta v*/

    /* Generating Outputs */
    double fmassr;              /* *o (kg)     Remaining fuel mass*/
    double thrust;              /* *o (N)      Thrust*/
    double vmass;               /* *o (kg)     Vehicle mass*/

    arma::mat IBBB;             /* *o (kg*m2)  Vehicle moment of inertia*/
    double _IBBB[3][3];         /* *o (kg*m2)  Vehicle moment of inertia*/

    double mass_ratio;
    double fuel_mass;
    double oxidizer_mass;

    arma::vec xcg;
    double _xcg[3];

    unsigned int CG_OFFSET;
    unsigned int TWD;

    /* Engine variables */

    double S2_E1_mass;          /* *o (kg)      S2 No.1 Engine mass */
    double S2_E1_mass_0;        /* *o (kg)      Initial S2 No.1 Engine mass */
    double S2_E1_mass_1;        /* *o (kg)      Final S2 No.1 Engine mass */
    double S2_E1_fuel_mass;     /* *o (kg)      S2 No.1 Engine fuel mass */
    double S2_E1_roll_0;        /* *o (kg*m2)     Initial S2 No.1 Engine roll MOI */
    double S2_E1_roll_1;        /* *o (kg*m2)     Final S2 No.1 Engine roll MOI */
    double S2_E1_pitch_0;       /* *o (kg*m2)     Initial S2 No.1 Engine pitch MOI */
    double S2_E1_pitch_1;       /* *o (kg*m2)     Final S2 No.1 Engine roll MOI */
    double S2_E1_yaw_0;         /* *o (kg*m2)     Initial S2 No.1 Engine yaw MOI */
    double S2_E1_yaw_1;         /* *o (kg*m2)     Final S2 No.1 Engine yaw MOI */
    arma::mat I_S2_E1;          /* *o (kg*m2)     S2 No.1 Engine MOI */
    double _I_S2_E1[3][3];      /* *o (kg*m2)     S2 No.1 Engine MOI */
    arma::mat I_S2_E1_0;        /* *o (kg*m2)     Initial S2 No.1 Engine MOI */
    double _I_S2_E1_0[3][3];    /* *o (kg*m2)     Initial S2 No.1 Engine MOI */
    arma::mat I_S2_E1_1;        /* *o (kg*m2)     Final S2 No.1 Engine MOI */
    double _I_S2_E1_1[3][3];    /* *o (kg*m2)     Final S2 No.1 Engine MOI */
    double S2_E1_xcg_0;         /* *o (m)     Initial S2 No.1 Engine XCG */
    double S2_E1_xcg_1;         /* *o (m)     Final S2 No.1 Engine XCG */
    double S2_E1_xcg;           /* *o (m)     S2 No.1 Engine XCG */


    double S2_E2_mass;          /* *o (kg)      S2 No.2 Engine mass */
    double S2_E2_mass_0;        /* *o (kg)      Initial S2 No.2 Engine mass */
    double S2_E2_mass_1;        /* *o (kg)      Final S2 No.2 Engine mass */
    double S2_E2_fuel_mass;     /* *o (kg)      S2 No.2 Engine fuel mass */
    double S2_E2_roll_0;        /* *o (kg*m2)     Initial S2 No.2 Engine roll MOI */
    double S2_E2_roll_1;        /* *o (kg*m2)     Final S2 No.2 Engine roll MOI */
    double S2_E2_pitch_0;       /* *o (kg*m2)     Initial S2 No.2 Engine pitch MOI */
    double S2_E2_pitch_1;       /* *o (kg*m2)     Final S2 No.2 Engine roll MOI */
    double S2_E2_yaw_0;         /* *o (kg*m2)     Initial S2 No.2 Engine yaw MOI */
    double S2_E2_yaw_1;         /* *o (kg*m2)     Final S2 No.2 Engine yaw MOI */
    arma::mat I_S2_E2;          /* *o (kg*m2)     S2 No.2 Engine MOI */
    double _I_S2_E2[3][3];      /* *o (kg*m2)     S2 No.2 Engine MOI */
    arma::mat I_S2_E2_0;        /* *o (kg*m2)     Initial S2 No.2 Engine MOI */
    double _I_S2_E2_0[3][3];    /* *o (kg*m2)     Initial S2 No.2 Engine MOI */
    arma::mat I_S2_E2_1;        /* *o (kg*m2)     Final S2 No.2 Engine MOI */
    double _I_S2_E2_1[3][3];    /* *o (kg*m2)     Final S2 No.2 Engine MOI */
    double S2_E2_xcg_0;         /* *o (m)     Initial S2 No.2 Engine XCG */
    double S2_E2_xcg_1;         /* *o (m)     Final S2 No.2 Engine XCG */
    double S2_E2_xcg;           /* *o (m)     S2 No.2 Engine XCG */

    double S2_E3_mass;          /* *o (kg)      S2 No.3 Engine mass */
    double S2_E3_mass_0;        /* *o (kg)      Initial S2 No.3 Engine mass */
    double S2_E3_mass_1;        /* *o (kg)      Final S2 No.3 Engine mass */
    double S2_E3_fuel_mass;     /* *o (kg)      S2 No.3 Engine fuel mass */
    double S2_E3_roll_0;        /* *o (kg*m2)     Initial S2 No.3 Engine roll MOI */
    double S2_E3_roll_1;        /* *o (kg*m2)     Final S2 No.3 Engine roll MOI */
    double S2_E3_pitch_0;       /* *o (kg*m2)     Initial S2 No.3 Engine pitch MOI */
    double S2_E3_pitch_1;       /* *o (kg*m2)     Final S2 No.3 Engine roll MOI */
    double S2_E3_yaw_0;         /* *o (kg*m2)     Initial S2 No.3 Engine yaw MOI */
    double S2_E3_yaw_1;         /* *o (kg*m2)     Final S2 No.3 Engine yaw MOI */
    arma::mat I_S2_E3;          /* *o (kg*m2)     S2 No.3 Engine MOI */
    double _I_S2_E3[3][3];      /* *o (kg*m2)     S2 No.3 Engine MOI */
    arma::mat I_S2_E3_0;        /* *o (kg*m2)     Initial S2 No.3 Engine MOI */
    double _I_S2_E3_0[3][3];    /* *o (kg*m2)     Initial S2 No.3 Engine MOI */
    arma::mat I_S2_E3_1;        /* *o (kg*m2)     Final S2 No.3 Engine MOI */
    double _I_S2_E3_1[3][3];    /* *o (kg*m2)     Final S2 No.3 Engine MOI */
    double S2_E3_xcg_0;         /* *o (m)     Initial S2 No.3 Engine XCG */
    double S2_E3_xcg_1;         /* *o (m)     Final S2 No.3 Engine XCG */
    double S2_E3_xcg;           /* *o (m)     S2 No.3 Engine XCG */

    double S2_E4_mass;          /* *o (kg)      S2 No.4 Engine mass */
    double S2_E4_mass_0;        /* *o (kg)      Initial S2 No.4 Engine mass */
    double S2_E4_mass_1;        /* *o (kg)      Final S2 No.4 Engine mass */
    double S2_E4_fuel_mass;     /* *o (kg)      S2 No.4 Engine fuel mass */
    double S2_E4_roll_0;        /* *o (kg*m2)     Initial S2 No.4 Engine roll MOI */
    double S2_E4_roll_1;        /* *o (kg*m2)     Final S2 No.4 Engine roll MOI */
    double S2_E4_pitch_0;       /* *o (kg*m2)     Initial S2 No.4 Engine pitch MOI */
    double S2_E4_pitch_1;       /* *o (kg*m2)     Final S2 No.4 Engine roll MOI */
    double S2_E4_yaw_0;         /* *o (kg*m2)     Initial S2 No.4 Engine yaw MOI */
    double S2_E4_yaw_1;         /* *o (kg*m2)     Final S2 No.4 Engine yaw MOI */
    arma::mat I_S2_E4;          /* *o (kg*m2)     S2 No.4 Engine MOI */
    double _I_S2_E4[3][3];      /* *o (kg*m2)     S2 No.4 Engine MOI */
    arma::mat I_S2_E4_0;        /* *o (kg*m2)     Initial S2 No.4 Engine MOI */
    double _I_S2_E4_0[3][3];    /* *o (kg*m2)     Initial S2 No.4 Engine MOI */
    arma::mat I_S2_E4_1;        /* *o (kg*m2)     Final S2 No.4 Engine MOI */
    double _I_S2_E4_1[3][3];    /* *o (kg*m2)     Final S2 No.4 Engine MOI */
    double S2_E4_xcg_0;         /* *o (m)     Initial S2 No.4 Engine XCG */
    double S2_E4_xcg_1;         /* *o (m)     Final S2 No.4 Engine XCG */
    double S2_E4_xcg;           /* *o (m)     S2 No.4 Engine XCG */

    arma::vec structure_XCG;
    double _structure_XCG[3];

    double S2_spi;
    double S2_structure_mass;
    double S2_propellant_mass;
    double S2_remaining_fuel_mass;
    double S2_fmasse;
    double S3_spi;
    double S3_structure_mass;
    double S3_propellant_mass;
    double S3_remaining_fuel_mass;
    double S3_fmasse;
    double faring_mass;
    double ignition_time;
    double S2_timer;
    double S3_timer;

    /* END */
    /* Non-propagating Diagnostic Variables */
    /* These can be deleted, but keep to remain trackable in trick simulator */
};

#endif  // __propulsion_HH__
