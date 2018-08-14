#ifndef math_utility_c__H
#define math_utility_c__H
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Math functions C Version)
LIBRARY DEPENDENCY:
      ((../src/math_utility_c.c)
        (../../cad/src/global_constants.c))
*******************************************************************************/


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif

extern const double ___PI;
extern const double __RAD;

int pol_from_cart_C(const gsl_vector *cart, gsl_vector *pol);
int build_psivg_thtvg_TM_C(const double psivg, const double thtvg, gsl_matrix *AMAT);
int build_psi_tht_phi_TM_C(const double psi, const double tht, const double phi, gsl_matrix *TM);
int Matrix2Quaternion_C(gsl_matrix *Matrix_in, gsl_vector *Quaternion_out);
int Quaternion2Matrix_C(const gsl_vector *Quaternion_in, gsl_matrix *Matrix_out);
int Quaternion_cross_C(const gsl_vector *Quaternion_in1, const gsl_vector *Quaternion_in2, gsl_vector *Quaternion_out);
int Quaternion2Euler_C(const gsl_vector *Quaternion_in, double *Roll, double *Pitch, double *Yaw);
int Euler2Quaternion_C(const double Roll, const double Pitch, const double Yaw, gsl_vector *Quaternion_out);
int QuaternionMultiply_C(const gsl_vector *Quaternion_in1, const gsl_vector *Quaternion_in2, gsl_vector *Quaternion_out);
int Quaternion_rotation_C(const gsl_vector *Vector_in, const gsl_vector *Quaternion_in, gsl_vector *Vector_out);
int build_321_rotation_matrix(const gsl_vector *angle, gsl_matrix *mat_out);
int sign(const double variable);

gsl_matrix *skew_sym_C(const gsl_vector *vec);
gsl_vector *Quaternion_conjugate_C(const gsl_vector *Quaternion_in);
gsl_vector *QuaternionInverse_C(const gsl_vector *Quaternion_in);
#ifdef __cplusplus
}
#endif
#endif