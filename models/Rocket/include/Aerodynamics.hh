#ifndef __aerodynamics_HH__
#define __aerodynamics_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the AERODYNAMICS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Aerodynamics.cpp))
PROGRAMMERS:
      (((Jun-Xu Lai) () () () ))
*******************************************************************************/

#include "utility_header.hh"
#include "Euler.hh"
#include "Tvc.hh"
#include "Environment.hh"
#include "Propulsion.hh"

class TVC;
class _Euler_;
class Propulsion;
class Kinematics;

class AeroDynamics
{
    public:

        Kinematics *kinematics;
        Environment *environment;
        Propulsion *propulsion;
        _Euler_ *euler;
        Newton *newton;
        TVC *tvc;
        Datadeck aerotable; /* ** (--) Aero Deck */
        void initialize(Kinematics *kine, Environment *env, Propulsion *prop, _Euler_ *eul, Newton *newt,TVC *t);
        void calculate_aero(double int_step, Datadeck &aerotable);
        void aerodynamics_der();
    ///////////////////////////////////////////////////////////////////////////////
    //Definition of aerodynamic module-variables
    //Member function of class 'FSW'
    //Module-variable locations are assigned to FSW[100-199]
    //
    // This module performs the following functions:
    // (1) Calculates the aerodynamic force and moment coefficients from the aero_deck
    // (2) Derives the dimensional derivatives for the flight controllers
    //
    // maero = 0 no aerodynamic forces
    //         11 Booster - 1 stage (last stage)
    //         12 Booster - 2 stages
    //         13 Booster - 3 stages (launch)
    //
    //050103 Created by Peter H Zipfel
    //091214 Modified for ROCKET6, PZi
    ///////////////////////////////////////////////////////////////////////////////

    /***********************************Variables describtion******************************/
        int maero;          /* *io (--)     =11: last stage; =12: 2 stages; =13: 3 stages*/
        double refa;        /* *io (m2)     Reference area for aero coefficients - m^2*/
        double refd;        /* *io (m)      Reference length for aero coefficients - m*/
        double xcg_ref;     /* *io (m)      Reference cg location from nose - m*/
        double cy;          /* *io (--)     Side force coefficient - ND*/
        double cll;         /* *io (--)     Rolling moment coefficient - ND*/
        double clm;         /* *io (--)     Pitching moment coefficient - ND*/
        double cln;         /* *io (--)     Yawing moment coefficient - ND*/
        double cx;          /* *io (--)     X-force coefficient - ND*/
        double cz;          /* *io (--)     Z-force coefficient - ND*/
        double ca0;         /* *io (--)     Axial force coeff(Mach) - ND*/
        double caa;         /* *io (--)     Delta axial force due to alpha(Mach) - ND*/
        double cn0;         /* *io (--)     Normal force coeff(Mach,alpha) - ND*/
        double clm0;        /* *io (--)     Pitch moment coeff(Mach,alpha) - ND*/
        double clmq;        /* *io (1/d)    Pitch dampning deriv(Mach) - 1/deg*/
        double cla;         /* *io (1/d)    Lift slope derivative(alpha,mach) - 1/deg*/
        double clde;        /* *io (1/d)    Lift force due to elevator (alpha.mach), - 1/deg*/
        double cyb;         /* *io (1/d)    Weather vane der wrt beta(alpha,mach) - 1/deg*/
        double cydr;        /* *io (1/d)    Side force due to rudder deriv(alpha,mach) - 1/deg*/
        double cllda;       /* *io (1/d)    Roll control effectiveness(alpha,mach), - 1/deg*/
        double cllp;        /* *io (1/r)    Roll damping deriv(alpha,mach) - 1/rad*/
        double cma;         /* *io (1/d)    Pitch moment due to alpha deriv(alpha,mach) -1/deg*/
        double cmde;        /* *io (1/d)    Pitch control effectiveness(alpha,mach), - 1/deg*/
        double cmq;         /* *io (1/r)    Pitch dampning deriv(alpha,mach) - 1/rad*/
        double cnb;         /* *io (1/d)    Yaw moment deriv(alpha,mach) - 1/deg*/
        double cndr;        /* *io (1/d)    Yaw moment due to rudder deriv(alpha,mach) - 1/deg*/
        double cnr;         /* *io (1/r)    Yaw damping deriv(alpha,mach) - 1/rad*/
        double stmarg_yaw;  /* *io (--)     Static margin yaw (+stable, -unstable) - caliber*/
        double stmarg_pitch;/* *io (--)     Static margin pitch (+stable, -unstable) - caliber*/
        double dla;         /* *io (m/s2)   Lift slope derivative - m/s^2*/
        double dlde;        /* *io (m/s2)   Lift elevator control derivative - m/s^2*/
        double dma;         /* *io (1/s2)   Pitch moment derivative - 1/s^2*/
        double dmq;         /* *io (1/s)    Pitch damping derivative - 1/s*/
        double dmde;        /* *io (1/s2)   Pitch control derivative - 1/s^2*/
        double dyb;         /* *io (m/s2)   Side force derivative - m/s^2*/
        double dydr;        /* *io (m/s2)   Side force control derivative - m/s^2*/
        double dnb;         /* *io (1/s2)   Yawing moment derivative - 1/s^2*/
        double dnr;         /* *io (1/s)    Yaw dampnig derivative - 1/s*/
        double dndr;        /* *io (1/s2)   Yaw control derivative - 1/s^2*/
        double dllp;        /* *io (1/s)    Roll damping derivative - 1/s*/
        double dllda;       /* *io (1/s2)   Roll control derivative - 1/s^2*/
        double dnd;         /* *io (m/s2)   pitch contrl force derivative -m/s^2*/
        double realp1;      /* *io (r/s)    First real root of airframe pitch dyn  - rad/s*/
        double realp2;      /* *io (r/s)    Second real root of airframe pitch dyn - rad/s*/
        double wnp;         /* *io (r/s)    Natural frequency of airframe pitch dynamics - rad/s*/
        double zetp;        /* *io (--)     Damping of airframe pitch dynamics - NA*/
        double rpreal;      /* *io (r/s)    Real part or mean value (real roots) of pitch  - rad/s*/
        double realy1;      /* *io (r/s)    First real root of airframe yaw dynamics - rad/s*/
        double realy2;      /* *io (r/s)    Second real root of airframe yaw dynamics - rad/s*/
        double wny;         /* *io (r/s)    Natural frequency of airframe yaw dynamics - rad/s*/
        double zety;        /* *io (--)     Damping of airframe yaw dynamics - NA*/
        double ryreal;      /* *io (r/s)    Real part or mean value (real roots) of yaw - rad/s*/
        double alplimx;     /* *io (d)      Alpha limiter for vehicle - deg*/
        double alimitx;     /* *io (--)     Structural  limiter for vehicle*/
        double gnavail;     /* *io (--)     G available in pitch for vehicle - g's*/
        double gyavail;     /* *io (--)     G available in yaw for vehicle - g's*/
        double gnmax;       /* *io (--)     Max g permissable in pitch for vehicle - g's*/
        double gymax;       /* *io (--)     Max g permissable in yaw for vehicle - g's*/

};


#endif  // __aerodynamics_HH__
