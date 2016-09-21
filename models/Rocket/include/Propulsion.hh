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
#include "utility_header.hh"
#include "Environment.hh"

class Environment;

class Propulsion{
    public:
        Propulsion(){};
        void initialize(Environment *env);
        void calculate_propulsion(double int_step);
        Environment *environment;
        /***********************************Variables describtion******************************/
        int mprop;          /* *io (--)     propulsion mode =0:none; =3 input; =4 LTG control*/
        double acowl;       /* *io (m2)     Cowl area of engine inlet*/
        double vmass;       /* *io (kg)     Vehicle mass*/
        double vmass0;      /* *io (kg)     Initial vehicle mass*/
        double xcg;         /* *io (m)      Center 0f Gravity location from nose cone*/
        double ibbb[3][3];  /* *io (kg*m2)  Vehicle moment of inertia*/
        double fmass0;      /* *io (kg)     Initial fuel mass in stage*/
        double fmasse;      /* *io (kg)     Fuel mass expended (zero initialization required)*/
        double fmassd;      /* *io (kg/s)   Fuel mass expended derivative*/
        double aexit;       /* *io (m2)     Nozzle exit area*/
        double spi;         /* *io (s)      Specific impulse*/
        double thrust;      /* *io (N)      Thrust*/
        double fmassr;      /* *io (kg)     Remaining fuel mass*/
        double xcg_0;       /* *io (m)      Initial cg location from nose*/
        double xcg_1;       /* *io (m)      Final cg location from nose*/
        double fuel_flow_rate;/* *io (kg/s) Fuel flow rate of rocket motor*/
        double moi_roll_0;  /* *io (kg*m2)  Roll MOI of vehicle, initial*/
        double moi_roll_1;  /* *io (kg*m2)  Roll MOI of vehicle, burn-out*/
        double moi_trans_0; /* *io (kg*m2)  Transverse MOI of vehicle, initial*/
        double moi_trans_1; /* *io (kg*m2)  Transverse MOI of vehicle, burn-out*/
        double thrust_delta_v;/* *io (m/s)  delta v*/
        double payload;     /* *io (kg)     payload mass*/
        /**************************************************************************************/
};
#endif  // __propulsion_HH__
