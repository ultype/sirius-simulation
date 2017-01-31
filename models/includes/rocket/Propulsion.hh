#ifndef __propulsion_HH__
#define __propulsion_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the propulsion Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Propulsion.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "aux/global_constants.hh"
#include "aux/utility_header.hh"
#include "Environment.hh"

class Environment;

class Propulsion{
    TRICK_INTERFACE(Propulsion);

    public:
        Propulsion(Environment& env);
        Propulsion(const Propulsion& other);

        Propulsion& operator=(const Propulsion& other);

        void initialize();
        void calculate_propulsion(double int_step);

        void set_no_thrust();
        void set_input_thrust(double xcg0, double xcg1,
                                double moi_roll0, double moi_roll1,
                                double moi_trans0, double moi_tran1,
                                double spi, double fuel_flow_rate);
        void set_ltg_thrust();

        //XXX: get_thrust_state
        int get_mprop();
        double get_vmass();
        double get_xcg();
        double get_thrust();
        double get_fmassr();
        Matrix get_IBBB();

        /* Input File */
        void set_vmass0(double);
        void set_fmass0(double);

        void set_aexit(double);
        void set_payload(double);

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */
        void propagate_thrust_delta_v(double int_step);
        void propagate_fmasse(double int_step, double press, double psl);

        /* Internal Calculators */
        double calculate_thrust(double press, double psl);
        double calculate_fmassr();

        double calculate_xcg();

        /* Routing references */
        Environment * environment;

        /* Constants */
        typedef enum {
            NO_THRUST = 0,
            INPUT_THRUST = 3,
            LTG_THRUST = 4
        } THRUST_TYPE;

        double xcg_0;           /* *o (m)      Initial cg location from nose*/
        double xcg_1;           /* *o (m)      Final cg location from nose*/
        double moi_roll_0;      /* *o (kg*m2)  Roll MOI of vehicle, initial*/
        double moi_roll_1;      /* *o (kg*m2)  Roll MOI of vehicle, burn-out*/
        double moi_trans_0;     /* *o (kg*m2)  Transverse MOI of vehicle, initial*/
        double moi_trans_1;     /* *o (kg*m2)  Transverse MOI of vehicle, burn-out*/
        double fuel_flow_rate;  /* *o (kg/s)   Fuel flow rate of rocket motor*/
        double spi;             /* *o (s)      Specific impulse*/

        double aexit;           /* *o (m2)     Nozzle exit area*/
        double payload;         /* *o (kg)     payload mass*/

        double vmass0;          /* *o (kg)     Initial vehicle mass*/
        double fmass0;          /* *o (kg)     Initial fuel mass in stage*/

        /* State */
        THRUST_TYPE thrust_state;   /* *o (--)     Propulsion mode, See THRUST TYPE*/

        /* Propagative Stats */
        double fmasse;              /* *o (kg)     Fuel mass expended (zero initialization required)*/
        double fmassd;              /* *o (kg/s)   Fuel mass expended derivative*/
        double thrust_delta_v;      /* *o (m/s)    delta v*/

        /* Generating Outputs */
        double fmassr;              /* *o (kg)     Remaining fuel mass*/
        double thrust;              /* *o (N)      Thrust*/
        double vmass;               /* *o (kg)     Vehicle mass*/

        double xcg;                 /* *o (m)      Center 0f Gravity location from nose cone*/
        double ibbb[3][3];          /* *o (kg*m2)  Vehicle moment of inertia*/

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
};
#endif  // __propulsion_HH__
