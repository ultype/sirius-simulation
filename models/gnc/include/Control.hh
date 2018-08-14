#ifndef __CONTROL_HH__
#define __CONTROL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the CONTROL Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Control.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/
#include <armadillo>
#include "aux.hh"
#include "matrix/utility.hh"
#include "math_utility.hh"

class Control {
    TRICK_INTERFACE(Control);

 public:
        Control();
        Control(const Control& other);

        Control& operator=(const Control& other);

        void initialize();

        void control(double int_step);
        // double control_normal_accel(double ancomx, double int_step);
        // double control_yaw_accel(double alcomx, double int_step);
        // double control_pitch_rate(double qqdx);
        // double control_gamma(double thtvdcomx);
        void set_thtvdcomx(double in);
        // void set_maut(double in);
        void set_delimx(double in);
        void set_drlimx(double in);
        void set_pgam(double in);
        void set_wgam(double in);
        void set_zgam(double in);
        void pitch_down(double pitchcmd, double int_step);
        void roll_control(double rollcmd, double int_step);
        void yaw_control(double yawcmd, double int_step);
        void AOA_control(double aoacmd, double int_step);
        void S2_B_pseudo_G(arma::vec3 cmd, double int_step);
        void S3_B_pseudo_G(arma::vec3 cmd, double int_step);

        void set_IBBB0(double in1, double in2, double in3);
        void set_IBBB1(double in1, double in2, double in3);
        void set_controller_var(double in1, double in2, double in3, double in4, double in5, double in6);
        void set_S2_PITCH_DOWN_I();
        void set_S2_PITCH_DOWN_II();
        void set_S3_PITCH_DOWN();
        void set_S2_ROLL_CONTROL();
        void set_NO_CONTROL();
        void set_S2_AOA();
        void set_S3_AOA();
        void set_kpp(double in);
        void set_kpi(double in);
        void set_kppp(double in);
        void set_krp(double in);
        void set_kri(double in);
        void set_krpp(double in);
        void set_kyp(double in);
        void set_kyi(double in);
        void set_kypp(double in);
        void set_attcmd(double in1, double in2, double in3);
        void set_kaoap(double in);
        void set_kaoai(double in);
        void set_kaoapp(double in);
        void set_kaoad(double in);
        void set_aoacmd(double in);

        void get_control_gain(double in1, double in2, double in3, double in4, double in5, double in6, double in7, double in8, double in9, double in10, double in11, double in12,
                                double in13, double in14, double in15, double in16, double in17, double in18, double in19, double in20);


        double get_theta_a_cmd();
        double get_theta_b_cmd();
        double get_theta_c_cmd();
        double get_theta_d_cmd();
        void set_ierror_zero();
        void set_reference_point(double in);
        void set_engine_d(double in);

        void pitch_down_test(double pitchcmd, double int_step);

        arma::vec3 euler_angle(arma::mat33 TBD);


        enum CONTROL_TYPE {
            NO_CONTROL = 0,
            S2_PITCH_DOWN_I,
            S2_PITCH_DOWN_II,
            S2_ROLL_CONTROL,
            S3_PITCH_DOWN,
            S2_AOA,
            S3_AOA
        };
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

        // std::function<int()>    grab_mprop;

        // std::function<double()> grab_dvbe;

        // std::function<double()> grab_gymax;
        // std::function<double()> grab_dyb;
        // std::function<double()> grab_dnb;
        // std::function<double()> grab_dnr;
        // std::function<double()> grab_dndr;
        // std::function<double()> grab_dla;
        // std::function<double()> grab_dma;
        // std::function<double()> grab_dmq;
        // std::function<double()> grab_dmde;
        // std::function<double()> grab_dnd;
        // std::function<double()> grab_dlde;

        // std::function<double()> grab_pdynmc;

        std::function<double()> grab_dvbec;
        std::function<double()> grab_thtvdcx;
        std::function<double()> grab_thtbdcx;
        std::function<double()> grab_phibdcx;
        std::function<double()> grab_psibdcx;
        std::function<double()> grab_alphacx;
        std::function<double()> grab_altc;

        std::function<double()> grab_qqcx;
        std::function<double()> grab_rrcx;

        std::function<arma::vec3()> grab_FSPCB;

        std::function<arma::vec3()> grab_computed_WBIB;
        std::function<arma::vec4()> grab_TBDQ;
        std::function<arma::mat33()> grab_TBD;
        std::function<arma::mat33()> grab_TBICI;
        std::function<arma::mat33()> grab_TBIC;

        double  get_delecx();
        double  get_delrcx();

        arma::mat33 build_321_rotation_matrix(arma::vec3 angle);
        void Quaternion_cmd(double int_step);
        void calculate_xcg_thrust(double int_step);

 private:
        void default_data();

        double  delecx;         /* *io (d)      Pitch command deflection */  // n
        double  delrcx;         /* *io (d)      Yaw command deflection */    // n

        enum CONTROL_TYPE maut;           /* *io (--)     maut=|mauty|mautp| see table */
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

        double fmasse;
        double mdot;
        double fmass0;
        double xcg_0;
        double xcg_1;
        double isp;

        arma::vec IBBB0;
        double _IBBB0[3];

        arma::vec IBBB1;
        double _IBBB1[3];

        arma::vec IBBB2;
        double _IBBB2[3];

        arma::vec CONTROLCMD;
        double _CONTROLCMD[3];

        arma::vec CMDQ;
        double _CMDQ[4];

        arma::vec TCMDQ;
        double _TCMDQ[4];

        // pitch
        double thterror;
        double perrori;
        double perrorp;
        double perror_old;
        double pitchiout_old;
        double pN;
        double pdout_old;
        double kpp;
        double kpi;
        double kpd;
        double kppp;
        // /////////test////////////
        double thterrort;
        double perrorit;
        double perrorpt;
        double perror_oldt;
        double pitchiout_oldt;
        double pdout_oldt;


        // roll
        double rollerror;
        double rerrori;
        double rerrorp;
        double rerror_old;
        double rolliout_old;
        double rN;
        double rdout_old;
        double krp;
        double kri;
        double krd;
        double krpp;


        // yaw
        double yawerror;
        double yerrori;
        double yerrorp;
        double yerror_old;
        double yawiout_old;
        double yN;
        double ydout_old;
        double kyp;
        double kyi;
        double kyd;
        double kypp;




        // aoa
        double aoaerror;
        double aoaerrori;
        double aoaerrorp;
        double aoaerror_old;
        double aoaiout_old;
        double aoaN;
        double aoadout_old;
        double kaoap;
        double kaoai;
        double kaoad;
        double kaoapp;
        double aoaerror_smooth_old;
        double aoaerror_new;
        double aoaerror_out_old;


        double theta_a_cmd;
        double theta_b_cmd;
        double theta_c_cmd;
        double theta_d_cmd;
        double lx;

        double rollcmd;
        double pitchcmd;
        double yawcmd;
        double aoacmd;

        double pitchcmd_new;
        double pitchcmd_old;
        double pitchcmd_out_old;

        arma::vec delta_euler;
        double _delta_euler[3];

        arma::vec euler;
        double _euler[3];

        double xcg;
        double thrust;
        double mass_ratio;

        arma::vec WBICBT;
        double _WBICBT[3];

        arma::vec CMDG;
        double _CMDG[3];

        double reference_point;
        double d;
};

#endif  // __CONTROL_HH__
