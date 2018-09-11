#ifndef __CONTROL_C_HH__
#define __CONTROL_C_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the CONTROL Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Control_c.c)
        (../../cad/src/global_constants.c))
PROGRAMMERS:
      (((CHUN-HSU LAI) () () () ))
*******************************************************************************/

#include "math_utility_c.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Global constant */
extern const double __AGRAV;
extern const double __RAD;
/********************************/
extern double fmasse;
extern double mdot;
extern double fmass0;
extern double xcg_0;
extern double xcg_1;
extern double isp;
extern double xcg;
extern double thrust;
extern double mass_ratio;
extern double reference_point;
extern double d;
extern double theta_a_cmd;
extern double theta_b_cmd;
extern double theta_c_cmd;
extern double theta_d_cmd;
extern double lx;
// pitch
extern double thterror;
extern double perrori;
extern double perrorp;
extern double perror_old;
extern double pitchiout_old;
extern double pN;
extern double pdout_old;
extern double kpp;
extern double kpi;
extern double kpd;
extern double kppp;
extern double pitchcmd;
extern double pitchcmd_new;
extern double pitchcmd_old;
extern double pitchcmd_out_old;
// roll
extern double rollerror;
extern double rerrori;
extern double rerrorp;
extern double rerror_old;
extern double rolliout_old;
extern double rN;
extern double rdout_old;
extern double krp;
extern double kri;
extern double krd;
extern double krpp;
extern double rollcmd;
// yaw
extern double yawerror;
extern double yerrori;
extern double yerrorp;
extern double yerror_old;
extern double yawiout_old;
extern double yN;
extern double ydout_old;
extern double kyp;
extern double kyi;
extern double kyd;
extern double kypp;
extern double yawcmd;
// aoa
extern double aoaerror;
extern double aoaerrori;
extern double aoaerrorp;
extern double aoaerror_old;
extern double aoaiout_old;
extern double aoaN;
extern double aoadout_old;
extern double kaoap;
extern double kaoai;
extern double kaoad;
extern double kaoapp;
extern double aoaerror_smooth_old;
extern double aoaerror_new;
extern double aoaerror_out_old;
extern double aoacmd;

extern gsl_vector *IBBB0;
extern gsl_vector *IBBB1;
extern gsl_vector *IBBB2;
extern gsl_vector *CONTROLCMD;
extern gsl_vector *CMDQ;
extern gsl_vector *TCMDQ;
extern gsl_vector *delta_euler;
extern gsl_vector *euler;
extern gsl_vector *WBICBT;

void calculate_xcg_thrust(const double int_step, const double fmass0, const double mdot, const double xcg_0, const double xcg_1
                            , const double isp, double *fmasse, double *mass_ratio, double *xcg, double *thrust);
void S3_B_pseudo_G(gsl_vector *cmd, gsl_vector *IBBB0, gsl_vector *IBBB1, gsl_vector *IBBB2, double *lx
                    , double *theta_a_cmd, double *theta_b_cmd, const double xcg, const double reference_point
                    , const double thrust, const double mass_ratio, const double int_step);
void S2_B_pseudo_G(gsl_vector *cmd, gsl_vector *IBBB0, gsl_vector *IBBB1, gsl_vector *IBBB2
                    , double *lx, double *theta_a_cmd, double *theta_b_cmd, double *theta_c_cmd
                    , double *theta_d_cmd, const double d, const double xcg, const double reference_point
                    , const double thrust, const double mass_ratio, const double int_step);
void Quaternion_cmd(const double int_step, double pitchcmd, double pitchcmd_old, double pitchcmd_out_old
                    , double pitchcmd_new, double rollcmd, double yawcmd, gsl_vector *TCMDQ, gsl_vector *CMDQ
                    , gsl_matrix *TLI, gsl_matrix *TBIC);
void pitch_down(const double int_step, double pitchcmd, double kpp, double kpi
                , double kpd, double pN, double perror_old, double pitchiout_old
                ,double pdout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD);
void roll_control(const double int_step, double rollcmd, double krp, double kri, double krd
                    , double rN, double rollerror, double rerror_old, double rolliout_old
                    , double rdout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD);
void yaw_control(const double int_step, double yawcmd, double kyp, double kyi, double kyd
                    , double yN, double yawerror, double yerror_old, double yawiout_old
                    , double ydout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD);
void AOA_control(const double int_step, double aoacmd, double kaoap, double kaoai, double kaoad
                    , double alphacx, double aoaerror, double aoaerror_old, double aoaiout_old
                    , double aoadout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD);


#ifdef __cplusplus
}
#endif
#endif