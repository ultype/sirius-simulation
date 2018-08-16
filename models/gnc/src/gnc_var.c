#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include "time_utility_c.h"

/** INS Variables **/
/* Matrix */
gsl_matrix *WEII;
gsl_matrix *TBIC;
gsl_matrix *TDCI;
gsl_matrix *TEIC;
gsl_matrix *TBD;
gsl_matrix *TBICI;
gsl_matrix *TLI;

/* Vector */
gsl_vector *EVBI;
gsl_vector *EVBID;
gsl_vector *ESBI;
gsl_vector *ESBID;
gsl_vector *RICI;
gsl_vector *RICID;
gsl_vector *TBIC_Q;
gsl_vector *TBIDC_Q;
gsl_vector *SBIIC;
gsl_vector *VBIIC;
gsl_vector *SBEEC;
gsl_vector *VBEEC;
gsl_vector *WBICI;
gsl_vector *EGRAVI;
gsl_vector *VBECD;
gsl_vector *INS_I_ATT_ERR;
gsl_vector *TESTV;
gsl_vector *TMP_old;
gsl_vector *VBIIC_old;
gsl_vector *POS_ERR;
gsl_vector *GRAVGI;
gsl_vector *TBDQ;
gsl_vector *VBIIC_old_old;

/* Double */
double dbic;
double dvbec;
double alphacx;
double betacx;
double thtvdcx;
double psivdcx;
double alppcx;
double phipcx;
double loncx;
double latcx;
double altc;
double phibdcx;
double thtbdcx;
double psibdcx;
double ins_pos_err;
double ins_vel_err;
double ins_tilt_err;
double ins_pose_err;
double ins_vele_err;
double ins_phi_err;
double ins_tht_err;
double ins_psi_err;

/* Unsigned int */
unsigned int gpsupdate;
unsigned int liftoff;
unsigned int ideal;

GPS_TIME gpstime;
UTC_TIME utctime;
/*******************/


/* Control variables */
double fmasse;
double mdot;
double fmass0;
double xcg_0;
double xcg_1;
double isp;
double xcg;
double thrust;
double mass_ratio;
double reference_point;
double d;
double theta_a_cmd;
double theta_b_cmd;
double theta_c_cmd;
double theta_d_cmd;
double lx;
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
double pitchcmd;
double pitchcmd_new;
double pitchcmd_old;
double pitchcmd_out_old;
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
double rollcmd;
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
double yawcmd;
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
double aoacmd;
/*********************/





























