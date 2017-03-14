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
            ar & _GAINFP;
            ar & gainp;
            ar & gainl;
            ar & gkp;
            ar & gkphi;
            ar & isetc2;
            ar & wacly;
            ar & zacly;
            ar & pacly;
            ar & gainy;
            ar & _GAINFY;
            ar & factwaclp;
            ar & factwacly;
            ar & alcomx;
            ar & ancomx;
            ar & qqdx;
            ar & grate;
        }

        Control();
        Control(const Control& other);

        Control& operator=(const Control& other);

        void initialize();

        void control(double int_step);
        double control_normal_accel(double ancomx, double int_step);
        double control_yaw_accel(double alcomx, double int_step);
        double control_pitch_rate(double qqdx);
        double control_gamma(double thtvdcomx);
        void set_thtvdcomx(double in);
        void set_maut(double in);
        void set_delimx(double in);
        void set_drlimx(double in);
        void set_pgam(double in);
        void set_wgam(double in);
        void set_zgam(double in);

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

        std::function<int()>    grab_mprop;

        std::function<double()> grab_dvbe;

        std::function<double()> grab_gymax;
        std::function<double()> grab_dyb;
        std::function<double()> grab_dnb;
        std::function<double()> grab_dnr;
        std::function<double()> grab_dndr;
        std::function<double()> grab_dla;
        std::function<double()> grab_dma;
        std::function<double()> grab_dmq;
        std::function<double()> grab_dmde;
        std::function<double()> grab_dnd;
        std::function<double()> grab_dlde;

        std::function<double()> grab_pdynmc;

        std::function<double()> grab_dvbec;
        std::function<double()> grab_thtvdcx;
        std::function<double()> grab_thtbdcx;

        std::function<double()> grab_qqcx;
        std::function<double()> grab_rrcx;

        std::function<arma::vec3()> grab_FSPCB;

        double  get_delecx();
        double  get_delrcx();

    private:
        void default_data();

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
        arma::vec GAINFP;       /* *io (--)     Feedback gains of pitch accel controller */
        double   _GAINFP[3];    /* *io (--)     Feedback gains of pitch accel controller */
        double  gainp;          /* *io (s2/m)   Proportional gain in pitch acceleration loop */
        double  gainl;          /* *io (--)     Gain in lateral acceleration loop */
        double  gkp;            /* *io (s)      Gain of roll rate feedback */
        double  gkphi;          /* *io (--)     Gain of roll angle feedback */
        double  isetc2;         /* *io (--)     Flag to print freeze variables */
        double  wacly;          /* *io (r/s)    Nat freq of accel close loop pole, yaw */
        double  zacly;          /* *io (--)     Damping of accel close loop pole, yaw */
        double  pacly;          /* *io (--)     Close loop real pole, yaw */
        double  gainy;          /* *io (--)     Gain in lateral acceleration loop */
        arma::vec GAINFY;       /* *io (--)     Feedback gains of yaw accel controller */
        double   _GAINFY[3];    /* *io (--)     Feedback gains of yaw accel controller */
        double  factwaclp;      /* *io (--)     Factor to mod 'waclp': waclp*(1+factwacl) */
        double  factwacly;      /* *io (--)     Factor to mod 'wacly': wacly*(1+factwacl) */
        double  alcomx;         /* *io (--)     Lateral (horizontal) acceleration command */
        double  ancomx;         /* *io (--)     Pitch (normal) acceleration command */
        double  qqdx;           /* *io (--)     Desired pitch rate", "control */
        double  grate;          /* *io (--)     grate */
        double gainff;          /* *io  (--)    gain */
        double thtvdcomx;        /* *io (d)      Flight path angle command */
        arma::mat GAINGAM;      /* *io  (--)    Gain */
        double _GAINGAM[3][1];  /* *io  (--)    Gain */
        double pgam;            /*  *io  (--)    Gain */
        double wgam;            /*  *io  (--)    Gain */
        double zgam;            /*  *io  (--)    Gain */
};

#endif  // __CONTROL_HH__
