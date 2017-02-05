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

#include "Newton.hh"
#include "Ins.hh"
#include "Propulsion.hh"
class Propulsion;

class Guidance {
    TRICK_INTERFACE(Guidance);

    public:
        Guidance() {}
        Guidance(const Guidance& other);

        Guidance& operator= (const Guidance& other);

        void default_data();
        void initialize(INS *i, Newton *ntn, Propulsion *plp);

        void guidance(double int_step);

        Matrix guidance_ltg(int &mprop, double int_step, double time_ltg);
        void guidance_ltg_tgo(double &tgop,
                                     Matrix &BURNTN,
                                     Matrix &L_IGRLN,
                                     Matrix &TGON,
                                     double &l_igrl,
                                     int &nstmax,
                                     double &tgo,
                                     int &nst,
                                     Matrix &TAUN,
                                     Matrix VEXN,
                                     Matrix BOTN,
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
                                      Matrix BURNTN,
                                      Matrix L_IGRLN,
                                      Matrix TGON,
                                      Matrix TAUN,
                                      Matrix VEXN,
                                      double l_igrl,
                                      double time_ltg);
        void guidance_ltg_trate(Matrix &ULAM,
                                       Matrix &LAMD,
                                       Matrix &RGO,
                                       int &ipas2_flag,
                                       Matrix VGO,
                                       double s_igrl,
                                       double q_igrl,
                                       double j_over_l,
                                       double lamd_limit,
                                       double vgom,
                                       double time_ltg,
                                       double tgo,
                                       double tgop,
                                       Matrix SDII,
                                       Matrix SBIIC,
                                       Matrix VBIIC,
                                       Matrix RBIAS,
                                       Matrix UD,
                                       Matrix UY,
                                       Matrix UZ,
                                       Matrix &RGRAV);
        void guidance_ltg_trate_rtgo(Matrix &RGO,
                                            Matrix &RGRAV,
                                            double tgo,
                                            double tgop,
                                            Matrix SDII,
                                            Matrix SBIIC,
                                            Matrix VBIIC,
                                            Matrix RBIAS,
                                            Matrix ULAM,
                                            Matrix UD,
                                            Matrix UY,
                                            Matrix UZ,
                                            double s_igrl);
        void guidance_ltg_pdct(Matrix &SPII,
                                      Matrix &VPII,
                                      Matrix &RGRAV,
                                      Matrix &RBIAS,
                                      Matrix LAMD,
                                      Matrix ULAM,
                                      double l_igrl,
                                      double s_igrl,
                                      double j_igrl,
                                      double q_igrl,
                                      double h_igrl,
                                      double p_igrl,
                                      double j_over_l,
                                      double qprime,
                                      Matrix SBIIC,
                                      Matrix VBIIC,
                                      Matrix RGO,
                                      double tgo);
        void guidance_ltg_crct(Matrix &SDII,
                                      Matrix &UD,
                                      Matrix &UY,
                                      Matrix &UZ,
                                      Matrix &VMISS,
                                      Matrix &VGO,
                                      double dbi_desired,
                                      double dvbi_desired,
                                      double thtvdx_desired,
                                      Matrix SPII,
                                      Matrix VPII,
                                      Matrix SBIIC,
                                      Matrix VBIIC);

        INS* ins;
        Newton *newton;
        Propulsion *propulsion;

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
        Matrix  get_UTBC();

        double  get_alphacomx();
        double  get_betacomx();

        void    set_degree(double, double);
    private:
        /* Propagative Stats */
        double  utbc[3];        /* *io  (--)        Commanded unit thrust vector in body coor */

        /* Diagnostic */
        double  utic[3];        /* *io  (--)        Commanded unit thrust vector in inertial coor */

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

        double  rbias[3];       /* *io  (m)         Range-to-be-gained bias */
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
        double  rgrav[3];       /* *io  (m)         Postion loss due to gravity */
        double  rgo[3];         /* *io  (m)         Range-to-go vector */
        double  vgo[3];         /* *io  (m/s)       Velocity still to be gained */
        double  sdii[3];        /* *io  (m)         Desired inertial position */
        double  ud[3];          /* *io  (--)        Unit vector of SPII and SDII */
        double  uy[3];          /* *io  (--)        Unit vector normal to traj plane */
        double  uz[3];          /* *io  (--)        Unit vector in traj plane, normal to SBBI_D */
        double  vgom;           /* *io  (m/s)       Velocity to be gained magnitude */
        double  tgo;            /* *io  (s)         Time to go to desired end state */
        int     nst;            /* *io  (--)        N-th stage number */
        double  ulam[3];        /* *io  (--)        Unit thrust vector in VGO direction */
        double  _lamd[3];       /* *io  (1/s)       Thrust vector turning rate */
        int     nstmax;         /* *io  (--)        //# of stages needed to meet end state */
        double  lamd;           /* *io  (1/s)       Magnitude of LAMD */
        double  dpd;            /* *io  (m)         Distance of the predicted from the desired end-point */
        double  dbd;            /* *io  (m)         Distance of vehicle from the desired end-point */
        double  ddb;            /* *io  (m)         Position error at BECO */
        double  dvdb;           /* *io  (m/s)       Distance of vehicle from the desired end-point */
        double  thtvddbx;       /* *io  (d)         Angle error at BECO */
};

#endif  // __GUIDANCE_HH__
