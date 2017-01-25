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
        Propulsion(){};
        void initialize(Environment *env);
        void calculate_propulsion(double int_step);
        Environment *environment;
        /***********************************Variables describtion******************************/

        void set_mprop(int);
        int get_mprop();
        double get_vmass();
        double get_xcg();
        double get_thrust();
        double get_fmassr();
        Matrix get_IBBB();

        /* Input File */
        void set_vmass0(double);
        void set_fmass0(double);
        void set_xcg_0(double);
        void set_xcg_1(double);
        void set_fuel_flow_rate(double);
        void set_moi_roll_0(double);
        void set_moi_roll_1(double);
        void set_moi_trans_0(double);
        void set_moi_trans_1(double);
        void set_aexit(double);
        void set_spi(double);
        void set_payload(double);
        /* Input File Event */
        void set_fmasse(double);

    private:
        int mprop;          /* *io (--)     propulsion mode =0:none; =3 input; =4 LTG control*/
        double vmass;       /* *io (kg)     Vehicle mass*/
        double xcg;         /* *io (m)      Center 0f Gravity location from nose cone*/
        double ibbb[3][3];  /* *io (kg*m2)  Vehicle moment of inertia*/
        double thrust;      /* *io (N)      Thrust*/
        double fmassr;      /* *io (kg)     Remaining fuel mass*/

        double fmasse;      /* *io (kg)     Fuel mass expended (zero initialization required)*/
        double vmass0;      /* *io (kg)     Initial vehicle mass*/
        double fmass0;      /* *io (kg)     Initial fuel mass in stage*/
        double xcg_0;       /* *io (m)      Initial cg location from nose*/
        double xcg_1;       /* *io (m)      Final cg location from nose*/
        double fuel_flow_rate;/* *io (kg/s) Fuel flow rate of rocket motor*/
        double moi_roll_0;  /* *io (kg*m2)  Roll MOI of vehicle, initial*/
        double moi_roll_1;  /* *io (kg*m2)  Roll MOI of vehicle, burn-out*/
        double moi_trans_0; /* *io (kg*m2)  Transverse MOI of vehicle, initial*/
        double moi_trans_1; /* *io (kg*m2)  Transverse MOI of vehicle, burn-out*/
        double aexit;       /* *io (m2)     Nozzle exit area*/
        double spi;         /* *io (s)      Specific impulse*/
        double payload;     /* *io (kg)     payload mass*/

        double acowl;       /* *io (m2)     Cowl area of engine inlet*/
        double fmassd;      /* *io (kg/s)   Fuel mass expended derivative*/
        double thrust_delta_v;/* *io (m/s)  delta v*/
        /**************************************************************************************/
};
#endif  // __propulsion_HH__