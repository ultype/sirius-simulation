#ifndef __CONTROL_HH__
#define __CONTROL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the CONTROL Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Control.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Environment.hh"
#include "Ins.hh"
#include "Propulsion.hh"
#include "Aerodynamics.hh"

class Newton;
class Environment;
class Propulsion;
class AeroDynamics;

class Control {
    TRICK_INTERFACE(Control);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & delecx;
            ar & delrcx;

            ar & maut;
            ar & mfreeze;
            ar & waclp;
            ar & zaclp;
            ar & paclp;
            ar & delimx;
            ar & drlimx;
            ar & yyd;
            ar & yy;
            ar & zzd;
            ar & zz;
            ar & alcomx_actual;
            ar & ancomx_actual;
            ar & gainfp;
            ar & gainp;
            ar & gainl;
            ar & gkp;
            ar & gkphi;
            ar & isetc2;
            ar & wacly;
            ar & zacly;
            ar & pacly;
            ar & gainy;
            ar & gainfy;
            ar & factwaclp;
            ar & factwacly;
            ar & alcomx;
            ar & ancomx;
            ar & qqdx;
            ar & grate;
        }

        Control() {}

        void default_data();
        void initialize(INS *i, Newton *ntn, Environment *env, Propulsion *plp, AeroDynamics *aero);

        void control(double int_step);
        double control_normal_accel(double ancomx, double int_step);
        double control_yaw_accel(double alcomx, double int_step);
        double control_pitch_rate(double qqdx);

        Environment *environment;
        AeroDynamics *aerodynamics;
        INS* ins;
        Newton *newton;
        Propulsion *propulsion;

        ///////////////////////////////////////////////////////////////////////////////
        // Definition of control module-variables
        // Member function of class 'Hyper'
        // Module-variable locations are assigned to hyper[500-599]
        //
        // maut = |mauty|mautp|
        //
        // mauty = 0 no control, fixed control surfaces
        //       = 5 yaw acceleration control for yaw-to-turn
        //
        // mautp = 0 no control, fixed control surfaces
        //       = 3 pitch acceleration control
        //
        // 030520 Created by Peter H Zipfel
        // 091214 Modified for ROCKET6, PZi
        ///////////////////////////////////////////////////////////////////////////////
        double  get_delecx();
        double  get_delrcx();

    private:
        double  delecx;         /* *io (d)      Pitch command deflection */ // n
        double  delrcx;         /* *io (d)      Yaw command deflection */   // n

        int     maut;           /* *io (--)     maut=|mauty|mautp| see table */
        int     mfreeze;        /* *io (--)     =0:Unfreeze; =1:Freeze; increment for more */
        double  waclp;          /* *io (r/s)    Nat freq of accel close loop complex pole */
        double  zaclp;          /* *io (--)     Damping of accel close loop complex pole */
        double  paclp;          /* *io (--)     Close loop real pole */
        double  delimx;         /* *io (d)      Pitch command limiter */
        double  drlimx;         /* *io (d)      Yaw command limiter */
        double  yyd;            /* *io (m/s2)   Yaw feed-forward derivative variable */
        double  yy;             /* *io (m/s)    Yaw feed-forward integration variable */
        double  zzd;            /* *io (m/s2)   Pitch feed-forward derivative variable */
        double  zz;             /* *io (m/s)    Pitch feed-forward integration variable */
        double  alcomx_actual;  /* *io (--)     Later accel com limited by 'betalimx' */
        double  ancomx_actual;  /* *io (--)     Normal accel com limited by 'alplimx' */
        double  gainfp[3];      /* *io (--)     Feedback gains of pitch accel controller */
        double  gainp;          /* *io (s2/m)   Proportional gain in pitch acceleration loop */
        double  gainl;          /* *io (--)     Gain in lateral acceleration loop */
        double  gkp;            /* *io (s)      Gain of roll rate feedback */
        double  gkphi;          /* *io (--)     Gain of roll angle feedback */
        double  isetc2;         /* *io (--)     Flag to print freeze variables */
        double  wacly;          /* *io (r/s)    Nat freq of accel close loop pole, yaw */
        double  zacly;          /* *io (--)     Damping of accel close loop pole, yaw */
        double  pacly;          /* *io (--)     Close loop real pole, yaw */
        double  gainy;          /* *io (--)     Gain in lateral acceleration loop */
        double  gainfy[3];      /* *io (--)     Feedback gains of yaw accel controller */
        double  factwaclp;      /* *io (--)     Factor to mod 'waclp': waclp*(1+factwacl) */
        double  factwacly;      /* *io (--)     Factor to mod 'wacly': wacly*(1+factwacl) */
        double  alcomx;         /* *io (--)     Lateral (horizontal) acceleration command */
        double  ancomx;         /* *io (--)     Pitch (normal) acceleration command */
        double  qqdx;           /* *io (--)     Desired pitch rate", "control */
        double  grate;          /* *io (--)     grate */
};

#endif  // __CONTROL_HH__
