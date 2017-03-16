#ifndef __GUIDANCE_HH__
#define __GUIDANCE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the GUIDANCE Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Guidance.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/
#include <armadillo>

#include "Newton.hh"
#include "Ins.hh"
#include "Propulsion.hh"
class Propulsion;

class Guidance {
    TRICK_INTERFACE(Guidance);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            /* Propagative Stats */
            ar & _UTBC;

            /* Diagnostic */
            ar & _UTIC;

            /* Set by input.py */
            ar & alphacomx;
            ar & betacomx;

            /* Internally set parameter */
            ar & init_flag;
            ar & inisw_flag;
            ar & skip_flag;
            ar & ipas_flag;
            ar & ipas2_flag;
            ar & print_flag;
            ar & time_ltg;

            ar & ltg_count;

            /* Externally set parameter */
            ar & mguide;
            ar & ltg_step;

            ar & _RBIAS;
            ar & beco_flag;
            ar & dbi_desired;
            ar & dvbi_desired;
            ar & thtvdx_desired;
            ar & num_stages;
            ar & delay_ignition;
            ar & amin;
            ar & char_time1;
            ar & char_time2;
            ar & char_time3;
            ar & exhaust_vel1;
            ar & exhaust_vel2;
            ar & exhaust_vel3;
            ar & burnout_epoch1;
            ar & burnout_epoch2;
            ar & burnout_epoch3;
            ar & lamd_limit;
            ar & _RGRAV;
            ar & _RGO;
            ar & _VGO;
            ar & _SDII;
            ar & _UD;
            ar & _UY;
            ar & _UZ;
            ar & vgom;
            ar & tgo;
            ar & nst;
            ar & ULAM;
            ar & LAMD;
            ar & nstmax;
            ar & lamd;
            ar & dpd;
            ar & dbd;
            ar & ddb;
            ar & dvdb;
            ar & thtvddbx;
        }

        Guidance();
        Guidance(const Guidance& other);

        Guidance& operator= (const Guidance& other);

        void default_data();
        void initialize();

        void guidance(double int_step);

        arma::vec3 guidance_ltg(int &mprop, double int_step, double time_ltg);
        void guidance_ltg_tgo(double &tgop,
                                     arma::mat &BURNTN,
                                     arma::mat &L_IGRLN,
                                     arma::mat &TGON,
                                     double &l_igrl,
                                     int &nstmax,
                                     double &tgo,
                                     int &nst,
                                     arma::mat &TAUN,
                                     arma::mat VEXN,
                                     arma::mat BOTN,
                                     double delay_ignition,
                                     double vgom,
                                     double amag1,
                                     double amin,
                                     double time_ltg,
                                     int num_stages);
        void guidance_ltg_igrl(double &s_igrl,
                                      double &j_igrl,
                                      double &q_igrl,
                                      double &h_igrl,
                                      double &p_igrl,
                                      double &j_over_l,
                                      double &tlam,
                                      double &qprime,
                                      int nst,
                                      int nstmax,
                                      arma::mat BURNTN,
                                      arma::mat L_IGRLN,
                                      arma::mat TGON,
                                      arma::mat TAUN,
                                      arma::mat VEXN,
                                      double l_igrl,
                                      double time_ltg);
        void guidance_ltg_trate(arma::mat &ULAM,
                                       arma::mat &LAMD,
                                       arma::mat &RGO,
                                       int &ipas2_flag,
                                       arma::mat VGO,
                                       double s_igrl,
                                       double q_igrl,
                                       double j_over_l,
                                       double lamd_limit,
                                       double vgom,
                                       double time_ltg,
                                       double tgo,
                                       double tgop,
                                       arma::mat SDII,
                                       arma::mat SBIIC,
                                       arma::mat VBIIC,
                                       arma::mat RBIAS,
                                       arma::mat UD,
                                       arma::mat UY,
                                       arma::mat UZ,
                                       arma::mat &RGRAV);
        void guidance_ltg_trate_rtgo(arma::mat &RGO,
                                            arma::mat &RGRAV,
                                            double tgo,
                                            double tgop,
                                            arma::mat SDII,
                                            arma::mat SBIIC,
                                            arma::mat VBIIC,
                                            arma::mat RBIAS,
                                            arma::mat ULAM,
                                            arma::mat UD,
                                            arma::mat UY,
                                            arma::mat UZ,
                                            double s_igrl);
        void guidance_ltg_pdct(arma::mat &SPII,
                                      arma::mat &VPII,
                                      arma::mat &RGRAV,
                                      arma::mat &RBIAS,
                                      arma::mat LAMD,
                                      arma::mat ULAM,
                                      double l_igrl,
                                      double s_igrl,
                                      double j_igrl,
                                      double q_igrl,
                                      double h_igrl,
                                      double p_igrl,
                                      double j_over_l,
                                      double qprime,
                                      arma::mat SBIIC,
                                      arma::mat VBIIC,
                                      arma::mat RGO,
                                      double tgo);
        void guidance_ltg_crct(arma::mat &SDII,
                                      arma::mat &UD,
                                      arma::mat &UY,
                                      arma::mat &UZ,
                                      arma::mat &VMISS,
                                      arma::mat &VGO,
                                      double dbi_desired,
                                      double dvbi_desired,
                                      double thtvdx_desired,
                                      arma::mat SPII,
                                      arma::mat VPII,
                                      arma::mat SBIIC,
                                      arma::mat VBIIC);

        std::function<int()>            grab_mprop;

        std::function<double()>         grab_dbi;
        std::function<double()>         grab_dvbi;
        std::function<double()>         grab_thtvdx;
        std::function<double()>         grab_fmassr;
        std::function<arma::vec3()>     grab_VBIIC;
        std::function<arma::vec3()>     grab_SBIIC;
        std::function<arma::mat33()>    grab_TBIC;
        std::function<arma::vec3()>     grab_FSPCB;

        std::function<void()>           set_no_thrust;
        std::function<void()>           set_ltg_thrust;

        ///////////////////////////////////////////////////////////////////////////////
        // Definition of guidance module-variables
        // Member function of class 'Hyper'
        // Module-variable locations are assigned to hyper[400-499]
        // Overflow assignment hyper[850-899]
        //
        //       mguide =  0 no guidance
        //              =  5 linear tangent guidance law (LTG) for rocket ascent
        //
        // 030616 Created by Peter H Zipfel
        // 091214 Modified for ROCKET6, PZi
        ///////////////////////////////////////////////////////////////////////////////
        arma::vec3  get_UTBC();

        double  get_alphacomx();
        double  get_betacomx();

        void    set_degree(double, double);
    private:
        /* Propagative Stats */
        arma::vec  UTBC;        /* *io  (--)        Commanded unit thrust vector in body coor */
        double  _UTBC[3];        /* *io  (--)        Commanded unit thrust vector in body coor */

        /* Diagnostic */
        arma::vec  UTIC;        /* *io  (--)        Commanded unit thrust vector in inertial coor */
        double  _UTIC[3];        /* *io  (--)        Commanded unit thrust vector in inertial coor */

        /* Set by input.py */
        double  alphacomx;      /* *io  (d)         Alpha command */
        double  betacomx;       /* *io  (d)         Beta command */

        /* Internally set parameter */
        int     init_flag;      /* *io  (--)        Flag for initializing LTG flags */
        int     inisw_flag;     /* *io  (--)        Flag to initialize '..._intl()' */
        int     skip_flag;      /* *io  (--)        Flag to delay output */
        int     ipas_flag;      /* *io  (--)        Flag to initialize in '..._tgo()'  */
        int     ipas2_flag;     /* *io  (--)        Flag to initialize in '..._trat()'  */
        int     print_flag;     /* *io  (--)        Flag to cause print-out  */
        double  time_ltg;       /* *io  (s)         Time since initiating LTG */

        int     ltg_count;      /* *io  (--)        Counter of LTG guidance cycles  */

        /* Externally set parameter */
        int     mguide;         /* *io  (--)        Guidance modes, see table */
        double  ltg_step;       /* *io  (s)         LTG guidance time step */

        arma::vec RBIAS;       /* *io  (m)         Range-to-be-gained bias */
        double   _RBIAS[3];       /* *io  (m)         Range-to-be-gained bias */
        int     beco_flag;      /* *io  (--)        Boost engine cut-off flag */
        double  dbi_desired;    /* *io  (m)         Desired orbital end position */
        double  dvbi_desired;   /* *io  (m/s)       Desired orbital end velocity */
        double  thtvdx_desired; /* *io  (d)         Desired orbital flight path angle */
        int     num_stages;     /* *io  (s)         Number of stages in boost phase */
        double  delay_ignition; /* *io  (s)         Delay of motor ignition after staging */
        double  amin;           /* *io  (m/s2)      Minimum longitudinal acceleration */
        double  char_time1;     /* *io  (s)         Characteristic time 'tau' of stage 1 */
        double  char_time2;     /* *io  (s)         Characteristic time 'tau' of stage 2 */
        double  char_time3;     /* *io  (s)         Characteristic time 'tau' of stage 3 */
        double  exhaust_vel1;   /* *io  (m/s)       Exhaust velocity of stage 1 */
        double  exhaust_vel2;   /* *io  (m/s)       Exhaust velocity of stage 2 */
        double  exhaust_vel3;   /* *io  (m/s)       Exhaust velocity of stage 3 */
        double  burnout_epoch1; /* *io  (s)         Burn out of stage 1 at 'time_ltg' */
        double  burnout_epoch2; /* *io  (s)         Burn out of stage 2 at 'time_ltg' */
        double  burnout_epoch3; /* *io  (s)         Burn out of stage 3 at 'time_ltg' */
        double  lamd_limit;     /* *io  (1/s)       Limiter on 'lamd' */
        arma::vec  RGRAV;       /* *io  (m)         Postion loss due to gravity */
        double  _RGRAV[3];       /* *io  (m)         Postion loss due to gravity */
        arma::vec  RGO;         /* *io  (m)         Range-to-go vector */
        double  _RGO[3];         /* *io  (m)         Range-to-go vector */
        arma::vec  VGO;         /* *io  (m/s)       Velocity still to be gained */
        double  _VGO[3];         /* *io  (m/s)       Velocity still to be gained */
        arma::vec  SDII;        /* *io  (m)         Desired inertial position */
        double  _SDII[3];        /* *io  (m)         Desired inertial position */
        arma::vec  UD;          /* *io  (--)        Unit vector of SPII and SDII */
        double  _UD[3];          /* *io  (--)        Unit vector of SPII and SDII */
        arma::vec  UY;          /* *io  (--)        Unit vector normal to traj plane */
        double  _UY[3];          /* *io  (--)        Unit vector normal to traj plane */
        arma::vec  UZ;          /* *io  (--)        Unit vector in traj plane, normal to SBBI_D */
        double  _UZ[3];          /* *io  (--)        Unit vector in traj plane, normal to SBBI_D */
        double  vgom;           /* *io  (m/s)       Velocity to be gained magnitude */
        double  tgo;            /* *io  (s)         Time to go to desired end state */
        int     nst;            /* *io  (--)        N-th stage number */
        arma::vec  ULAM;        /* *io  (--)        Unit thrust vector in VGO direction */
        double  _ULAM[3];        /* *io  (--)        Unit thrust vector in VGO direction */
        arma::vec  LAMD;       /* *io  (1/s)       Thrust vector turning rate */
        double  _LAMD[3];       /* *io  (1/s)       Thrust vector turning rate */
        int     nstmax;         /* *io  (--)        //# of stages needed to meet end state */
        double  lamd;           /* *io  (1/s)       Magnitude of LAMD */
        double  dpd;            /* *io  (m)         Distance of the predicted from the desired end-point */
        double  dbd;            /* *io  (m)         Distance of vehicle from the desired end-point */
        double  ddb;            /* *io  (m)         Position error at BECO */
        double  dvdb;           /* *io  (m/s)       Distance of vehicle from the desired end-point */
        double  thtvddbx;       /* *io  (d)         Angle error at BECO */
};

#endif  // __GUIDANCE_HH__
