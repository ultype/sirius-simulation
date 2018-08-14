#ifndef __INS_C_H__
#define __INS_C_H__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the INS Module On Board, Error equations based on Zipfel, Figure 10.27, space stabilized INS with GPS updates)
LIBRARY DEPENDENCY:
      ((../src/gnc_var.c)
        (../src/Ins_c.c)
        (../../cad/src/global_constants.c)
        (../src/dm_delta_ut.c))
*******************************************************************************/

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <math.h>
#include <stdio.h>
#include "cad_utility_c.h"
#include "math_utility_c.h"
#include "time_utility_c.h"

extern const double __DM_sec2r;
extern const double __DM_arcsec2r;
extern const double __WEII3;
extern const double __SMAJOR_AXIS;
extern const double __GM;
extern const double __WEII3;
extern const double ___PI;
extern const double __EPS;
extern const double __DEG;
/** INS Variables **/
/* Matrix */
extern gsl_matrix *WEII;
extern gsl_matrix *TBIC;
extern gsl_matrix *TDCI;
extern gsl_matrix *TEIC;
extern gsl_matrix *TBD;
extern gsl_matrix *TBICI;
extern gsl_matrix *TLI;

/* Vector */
extern gsl_vector *EVBI;
extern gsl_vector *EVBID;
extern gsl_vector *ESBI;
extern gsl_vector *ESBID;
extern gsl_vector *RICI;
extern gsl_vector *RICID;
extern gsl_vector *TBIC_Q;
extern gsl_vector *TBIDC_Q;
extern gsl_vector *SBIIC;
extern gsl_vector *VBIIC;
extern gsl_vector *SBEEC;
extern gsl_vector *VBEEC;
extern gsl_vector *WBICI;
extern gsl_vector *EGRAVI;
extern gsl_vector *VBECD;
extern gsl_vector *INS_I_ATT_ERR;
extern gsl_vector *TESTV;
extern gsl_vector *TMP_old;
extern gsl_vector *VBIIC_old;
extern gsl_vector *POS_ERR;
extern gsl_vector *GRAVGI;
extern gsl_vector *TBDQ;
extern gsl_vector *VBIIC_old_old;

/* Double */
extern double dbic;
extern double dvbec;
extern double alphacx;
extern double betacx;
extern double thtvdcx;
extern double psivdcx;
extern double alppcx;
extern double phipcx;
extern double loncx;
extern double latcx;
extern double altc;
extern double phibdcx;
extern double thtbdcx;
extern double psibdcx;
extern double ins_pos_err;
extern double ins_vel_err;
extern double ins_tilt_err;
extern double ins_pose_err;
extern double ins_vele_err;
extern double ins_phi_err;
extern double ins_tht_err;
extern double ins_psi_err;

/* Unsigned int */
extern unsigned int gpsupdate;
extern unsigned int liftoff;
extern unsigned int ideal;
/*******************/

int load_location(double lonx, double latx, double alt);
int load_angle(double yaw, double roll, double pitch, GPS_TIME gps_time);
int load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);
int calculate_INS_derived_TEI(GPS_TIME gps, gsl_matrix *TEIC);
int AccelHarmonic(const gsl_vector *SBII, double CS[21][21], int n_max, int m_max, const gsl_matrix *TEIC, gsl_vector *acc_out);
int DCM_2_Euler_angle(const gsl_matrix *TBD, double *phibdc, double *thtbdc, double *psibdc);
int calculate_INS_derived_phip(gsl_vector *VBECB, double *phipc);
int calculate_INS_derived_thtvd(gsl_vector *VBECD, double *thtvd);
int calculate_INS_derived_psivd(gsl_vector *VBECD, double *psivd);
int calculate_INS_derived_alpp(gsl_vector *VBECB, double *alpp);
int calculate_INS_derived_beta(gsl_vector *VBECB, double *beta);
int calculate_INS_derived_alpha(gsl_vector *VBECB, double *alpha);
int build_VBEB(double _alpha0x, double _beta0x, double _dvbe, gsl_vector *VBEB);
int INS_update(const double int_step, double *dvbec, unsigned int liftoff, double *alphacx, double *betacx
                , double *alppcx, double *phipcx, double *loncx, double *latcx, double *altc, double *psivdcx, double *thtvdcx
                , double *phibdc, double *thtbdc, double *psibdc
                , gsl_vector *WBICB, gsl_vector *PHI, gsl_vector *DELTA_VEL
                , gsl_vector *PHI_HIGH, gsl_vector *PHI_LOW, GPS_TIME gps, gsl_matrix *TEIC
                , gsl_vector *SBIIC, gsl_vector *GRAVGI, gsl_matrix *TBIC, gsl_vector *SBEEC
                , gsl_vector *VBEEC, gsl_matrix *WEII, gsl_matrix *TLI, gsl_matrix *TDCI
                , gsl_matrix *TBICI, gsl_matrix *TBDC, gsl_vector *TBDCQ);
#endif
