#ifndef __MATRIX_UTIL_HH__
#define __MATRIX_UTIL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Matrix utility functions)
LIBRARY DEPENDENCY:
      ((../../../src/math/matrix/utility.cpp))
*******************************************************************************/
#include <armadillo>

/** Returns polar from cartesian coordinates.
 * magnitude = POLAR(0,0) = |V|
 * azimuth   = POLAR(1,0) = atan2(V2,V1)
 * elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
 * Example: POLAR = VEC.pol_from_cart();
 */
arma::vec3 pol_from_cart(arma::vec3 in);

/// @return the angle between two 3x1 vectors
double angle(arma::vec3 VEC1, arma::vec3 VEC2);

/// @return skew symmetric matrix of a Vector3
arma::mat33 skew_sym(arma::vec3 vec);

/// @return the T.M. of the psivg -> thtvg sequence
arma::mat33 build_psivg_thtvg_TM(const double &psivg, const double &thtvg);

/// @return the Euler T.M. of the psi->tht->phi sequence
arma::mat33 build_psi_tht_phi_TM(const double &psi, const double &tht, const double &phi);

arma::vec4 Matrix2Quaternion(arma::mat33 Matrix_in);
arma::mat33 Quaternion2Matrix(arma::vec4 Quaternion_in);

#endif  // __MATRIX_UTIL_HH__
