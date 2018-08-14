#include "math_utility_c.h"

/**
*  Convert Cartesian coordinate system to Polar coordinate
*  @param[in] cart : cartseian system vector
*  @param[out] pol : Polar coordinate vector
*/
int pol_from_cart_C(const gsl_vector *cart, gsl_vector *pol) {
    double d = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double denom;

    double v1 = gsl_vector_get(cart, 0);
    double v2 = gsl_vector_get(cart, 1);
    double v3 = gsl_vector_get(cart, 2);

    d = gsl_blas_dnrm2(cart);
    azimuth = atan2(v2, v1);
    denom = sqrt(v1 * v1 + v2 * v2);
    if (denom > 0.) {
        elevation = atan2(-v3, denom);
    } else {
        if (v3 > 0)
            elevation = -___PI / 2.;
        if (v3 < 0)
            elevation = ___PI / 2.;
        if (v3 == 0)
            elevation = 0.;
    }

    gsl_vector_set(pol, 0, d);
    gsl_vector_set(pol, 1, azimuth);
    gsl_vector_set(pol, 2, elevation);

    return 0;
}


/*
*  Convert 3x1 vector into skew symmertric matrix
*  @param[in]  vec 
*  @param[out] mat
*           | 0 -c  b|        |a|
*           | c  0 -a| <--    |b|
*           |-b  a  0|        |c|
*/

gsl_matrix *skew_sym_C(const gsl_vector *vec) {
    gsl_matrix *mat;
    mat = gsl_matrix_calloc(3, 3);

    double v1 = gsl_vector_get(vec, 0);
    double v2 = gsl_vector_get(vec, 1);
    double v3 = gsl_vector_get(vec, 2);

    gsl_matrix_set(mat, 0, 0, 0.0);
    gsl_matrix_set(mat, 0, 1, -v3);
    gsl_matrix_set(mat, 0, 2, v2);
    gsl_matrix_set(mat, 1, 0, v3);
    gsl_matrix_set(mat, 1, 1, 0.0);
    gsl_matrix_set(mat, 1, 2, -v1);
    gsl_matrix_set(mat, 2, 0, -v2);
    gsl_matrix_set(mat, 2, 1, v1);
    gsl_matrix_set(mat, 2, 2, 0.0);

    return mat;
}
/*
*  Build psivg thtvg T.M.
* @param[in] psivg
* @param[in] thtvg
* @param[out] AMAT
*/
int build_psivg_thtvg_TM_C(const double psivg, const double thtvg, gsl_matrix *AMAT) {
    double sthtvg = sin(thtvg);
    double spsivg = sin(psivg);
    double cthtvg = cos(thtvg);
    double cpsivg = cos(psivg);

    gsl_matrix_set(AMAT, 0, 0, cthtvg * cpsivg);
    gsl_matrix_set(AMAT, 0, 1, -cthtvg * (-spsivg));
    gsl_matrix_set(AMAT, 0, 2, -sthtvg);
    gsl_matrix_set(AMAT, 1, 0, -spsivg);
    gsl_matrix_set(AMAT, 1, 1, cpsivg);
    gsl_matrix_set(AMAT, 1, 2, 0.0);
    gsl_matrix_set(AMAT, 2, 0, sthtvg * (cpsivg));
    gsl_matrix_set(AMAT, 2, 1, -sthtvg * (-spsivg));
    gsl_matrix_set(AMAT, 2, 2, cthtvg);

    return 0;
}

/*
*  Build T.M. of the 321 rotation sequence
* @param[in] psi : Yaw angle
* @parma[in] tht : Pitch angle
* @param[in] phi : Roll angle
* @param[out] TM
*/

int build_psi_tht_phi_TM_C(const double psi, const double tht, const double phi, gsl_matrix *TM) {
    double spsi = sin(psi);
    double cpsi = cos(psi);
    double stht = sin(tht);
    double ctht = cos(tht);
    double sphi = sin(phi);
    double cphi = cos(phi);

    gsl_matrix_set(TM, 0, 0, cpsi * ctht);
    gsl_matrix_set(TM, 0, 1, spsi * ctht);
    gsl_matrix_set(TM, 0, 2, -stht);
    gsl_matrix_set(TM, 1, 0, cpsi * stht * sphi - spsi * cphi);
    gsl_matrix_set(TM, 1, 1, spsi * stht * sphi + cpsi * cphi);
    gsl_matrix_set(TM, 1, 2, ctht * sphi);
    gsl_matrix_set(TM, 2, 0, cpsi * stht * cphi + spsi * sphi);
    gsl_matrix_set(TM, 2, 1, spsi * stht * cphi - cpsi * sphi);
    gsl_matrix_set(TM, 2, 2, ctht * cphi);

    return 0;
}

int Matrix2Quaternion_C(gsl_matrix *Matrix_in, gsl_vector *Quaternion_out) {
    gsl_vector *q_square;
    q_square = gsl_vector_calloc(4);
    double q_square_max;
    int j;
    gsl_matrix_transpose(Matrix_in);

    gsl_vector_set(q_square, 0, fabs(1.0 + gsl_matrix_get(Matrix_in, 0, 0) + gsl_matrix_get(Matrix_in, 1, 1) + gsl_matrix_get(Matrix_in, 2, 2)));
    gsl_vector_set(q_square, 1, fabs(1.0 + gsl_matrix_get(Matrix_in, 0, 0) - gsl_matrix_get(Matrix_in, 1, 1) - gsl_matrix_get(Matrix_in, 2, 2)));
    gsl_vector_set(q_square, 2, fabs(1.0 - gsl_matrix_get(Matrix_in, 0, 0) + gsl_matrix_get(Matrix_in, 1, 1) - gsl_matrix_get(Matrix_in, 2, 2)));
    gsl_vector_set(q_square, 3, fabs(1.0 - gsl_matrix_get(Matrix_in, 0, 0) - gsl_matrix_get(Matrix_in, 1, 1) + gsl_matrix_get(Matrix_in, 2, 2)));

    q_square_max = gsl_vector_max(q_square);
    j = gsl_vector_max_index(q_square);

    switch (j) {
        case 0:
            gsl_vector_set(Quaternion_out, 0, 0.5 * sqrt(q_square_max));
            gsl_vector_set(Quaternion_out, 1, 0.25 * (gsl_matrix_get(Matrix_in, 2, 1) - gsl_matrix_get(Matrix_in, 1, 2)) / gsl_vector_get(Quaternion_out, 0));
            gsl_vector_set(Quaternion_out, 2, 0.25 * (gsl_matrix_get(Matrix_in, 0, 2) - gsl_matrix_get(Matrix_in, 2, 0)) / gsl_vector_get(Quaternion_out, 0));
            gsl_vector_set(Quaternion_out, 3, 0.25 * (gsl_matrix_get(Matrix_in, 1, 0) - gsl_matrix_get(Matrix_in, 0 ,1)) / gsl_vector_get(Quaternion_out, 0));
            break;
        case 1:
            gsl_vector_set(Quaternion_out, 1, 0.5 * sqrt(q_square_max));
            gsl_vector_set(Quaternion_out, 0, 0.25 * (gsl_matrix_get(Matrix_in, 2, 1) - gsl_matrix_get(Matrix_in, 1, 2)) / gsl_vector_get(Quaternion_out, 1));
            gsl_vector_set(Quaternion_out, 2, 0.25 * (gsl_matrix_get(Matrix_in, 1, 0) - gsl_matrix_get(Matrix_in, 0, 1)) / gsl_vector_get(Quaternion_out, 1));
            gsl_vector_set(Quaternion_out, 3, 0.25 * (gsl_matrix_get(Matrix_in, 0, 2) - gsl_matrix_get(Matrix_in, 2, 0)) / gsl_vector_get(Quaternion_out, 1));
            break;
        case 2:
            gsl_vector_set(Quaternion_out, 2, 0.5 * sqrt(q_square_max));
            gsl_vector_set(Quaternion_out, 0, 0.25 * (gsl_matrix_get(Matrix_in, 0, 2) - gsl_matrix_get(Matrix_in, 2, 0)) / gsl_vector_get(Quaternion_out, 2));
            gsl_vector_set(Quaternion_out, 1, 0.25 * (gsl_matrix_get(Matrix_in, 1, 0) - gsl_matrix_get(Matrix_in, 0, 1)) / gsl_vector_get(Quaternion_out, 2));
            gsl_vector_set(Quaternion_out, 3, 0.25 * (gsl_matrix_get(Matrix_in, 2, 1) - gsl_matrix_get(Matrix_in, 1, 2)) / gsl_vector_get(Quaternion_out, 2));
            break;
        case 3:
            gsl_vector_set(Quaternion_out, 3, 0.5 * sqrt(q_square_max));
            gsl_vector_set(Quaternion_out, 0, 0.25 * (gsl_matrix_get(Matrix_in, 1, 0) - gsl_matrix_get(Matrix_in, 0, 1)) / gsl_vector_get(Quaternion_out, 3));
            gsl_vector_set(Quaternion_out, 1, 0.25 * (gsl_matrix_get(Matrix_in, 2, 0) - gsl_matrix_get(Matrix_in, 0, 2)) / gsl_vector_get(Quaternion_out, 3));
            gsl_vector_set(Quaternion_out, 2, 0.25 * (gsl_matrix_get(Matrix_in, 2, 1) - gsl_matrix_get(Matrix_in, 1, 2)) / gsl_vector_get(Quaternion_out, 3));
            break; 
        default:

            break;
    }
    gsl_vector_free(q_square);
    return 0;
}

int Quaternion2Matrix_C(const gsl_vector *Quaternion_in, gsl_matrix *Matrix_out) {
    gsl_matrix_set(Matrix_out, 0, 0, 2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 0)
                    + gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 1)) - 1.0);
    gsl_matrix_set(Matrix_out, 0, 1, 2.0 * (gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 2)
                    + gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 3)));
    gsl_matrix_set(Matrix_out, 0, 2, 2.0 * (gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 3)
                    - gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 2)));
    gsl_matrix_set(Matrix_out, 1, 0, 2.0 * (gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 2)
                    - gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 3)));
    gsl_matrix_set(Matrix_out, 1, 1, 2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 0)
                    + gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 2)) - 1.0);
    gsl_matrix_set(Matrix_out, 1, 2, 2.0 * (gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 3)
                    + gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 1)));
    gsl_matrix_set(Matrix_out, 2, 0, 2.0 * (gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 3)
                    + gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 2)));
    gsl_matrix_set(Matrix_out, 2, 1, 2.0 * (gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 3)
                    - gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 1)));
    gsl_matrix_set(Matrix_out, 2, 2, 2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 0)
                    + gsl_vector_get(Quaternion_in, 3) * gsl_vector_get(Quaternion_in, 3)) - 1.0);

    return 0;
}

gsl_vector *Quaternion_conjugate_C(const gsl_vector *Quaternion_in) {
    gsl_vector *Quaternion_out;
    Quaternion_out = gsl_vector_calloc(4);

    gsl_vector_set(Quaternion_out, 0, gsl_vector_get(Quaternion_in, 0));
    gsl_vector_set(Quaternion_out, 1, -gsl_vector_get(Quaternion_in, 1));
    gsl_vector_set(Quaternion_out, 2, -gsl_vector_get(Quaternion_in, 2));
    gsl_vector_set(Quaternion_out, 3, -gsl_vector_get(Quaternion_in, 3));

    return Quaternion_out;
}

int Quaternion_cross_C(const gsl_vector *Quaternion_in1, const gsl_vector *Quaternion_in2, gsl_vector *Quaternion_out) {
    gsl_vector_set(Quaternion_out, 0, gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 0) - gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 1) - gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 2) - gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 3));
    gsl_vector_set(Quaternion_out, 1, gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 0) + gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 1) - gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 2) + gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 3));
    gsl_vector_set(Quaternion_out, 2, gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 0) + gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 1) + gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 2) - gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 3));
    gsl_vector_set(Quaternion_out, 3, gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 0) - gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 1) + gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 2) + gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 3));

    return 0;
}

int Quaternion2Euler_C(const gsl_vector *Quaternion_in, double *Roll, double *Pitch, double *Yaw) {
    *Roll = atan2(2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 1) + gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 3)), 1.0 - 2.0 * (gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 1) + gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 2)));
    *Pitch = asin(2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 2) - gsl_vector_get(Quaternion_in, 3) * gsl_vector_get(Quaternion_in, 1)));
    *Yaw = atan2(2.0 * (gsl_vector_get(Quaternion_in, 0) * gsl_vector_get(Quaternion_in, 3) + gsl_vector_get(Quaternion_in, 1) * gsl_vector_get(Quaternion_in, 2)), 1.0 - 2.0 * (gsl_vector_get(Quaternion_in, 2) * gsl_vector_get(Quaternion_in, 2) + gsl_vector_get(Quaternion_in, 3) * gsl_vector_get(Quaternion_in, 3)));

    return 0;
}

int Euler2Quaternion_C(const double Roll, const double Pitch, const double Yaw, gsl_vector *Quaternion_out) {
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);

    gsl_vector_set(Quaternion_out, 0, cy * cr * cp + sy * sr * sp);
    gsl_vector_set(Quaternion_out, 1, cy * sr * cp - sy * cr * sp);
    gsl_vector_set(Quaternion_out, 2, cy * cr * sp + sy * sr * cp);
    gsl_vector_set(Quaternion_out, 3, sy * cr * cp - cy * sr * sp);

    return 0;
}

int QuaternionMultiply_C(const gsl_vector *Quaternion_in1, const gsl_vector *Quaternion_in2, gsl_vector *Quaternion_out) {
    gsl_vector_set(Quaternion_out, 0, gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 0) - gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 1)
                     - gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 2) - gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 3));
    gsl_vector_set(Quaternion_out, 1, gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 1) + gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 0)
                     + gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 3) - gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 2));
    gsl_vector_set(Quaternion_out, 2, gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 2) - gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 3)
                     + gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 0) + gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 1));
    gsl_vector_set(Quaternion_out, 3, gsl_vector_get(Quaternion_in1, 0) * gsl_vector_get(Quaternion_in2, 3) + gsl_vector_get(Quaternion_in1, 1) * gsl_vector_get(Quaternion_in2, 2)
                     - gsl_vector_get(Quaternion_in1, 2) * gsl_vector_get(Quaternion_in2, 1) + gsl_vector_get(Quaternion_in1, 3) * gsl_vector_get(Quaternion_in2, 0));

    return 0;
}

gsl_vector *QuaternionInverse_C(const gsl_vector *Quaternion_in) {
    gsl_vector *Quaternion_out;
    Quaternion_out = gsl_vector_calloc(4);

    Quaternion_out = Quaternion_conjugate_C(Quaternion_in);
    gsl_vector_scale(Quaternion_out, 1.0 / gsl_blas_dnrm2(Quaternion_in));

    return Quaternion_out;
}

int Quaternion_rotation_C(const gsl_vector *Vector_in, const gsl_vector *Quaternion_in, gsl_vector *Vector_out) {

    gsl_vector *Quaternion_temp;
    Quaternion_temp = gsl_vector_calloc(4);
    gsl_vector *Quaternion_temp2;
    Quaternion_temp2 = gsl_vector_calloc(4);

    for(int i = 0; i < 3; i++) gsl_vector_set(Quaternion_temp, i + 1, gsl_vector_get(Vector_in, i));  // R = (0, R)  R is vector in original reference frame, written as a quaternion with zero scalar part

    QuaternionMultiply_C(Quaternion_temp, Quaternion_in, Quaternion_temp2);  // r = q* x R x q
    QuaternionMultiply_C(Quaternion_conjugate_C(Quaternion_in), Quaternion_temp2, Quaternion_temp);  // r = q* x R x q

    for(int i = 0; i < 3; i++) gsl_vector_set(Vector_out, i, gsl_vector_get(Quaternion_temp, i + 1));

    gsl_vector_free(Quaternion_temp);
    gsl_vector_free(Quaternion_temp2);

    return 0;
}

int build_321_rotation_matrix(const gsl_vector *angle, gsl_matrix *mat_out) {
    double Roll, Pitch, Yaw;
    Roll = gsl_vector_get(angle, 0);
    Pitch = gsl_vector_get(angle, 1);
    Yaw = gsl_vector_get(angle, 2);

    gsl_matrix_set(mat_out, 0, 0, cos(Yaw) * cos(Pitch));
    gsl_matrix_set(mat_out, 0, 1, sin(Yaw) * cos(Pitch));
    gsl_matrix_set(mat_out, 0, 2, -sin(Pitch));
    gsl_matrix_set(mat_out, 1, 0, (cos(Yaw) * sin(Pitch) * sin(Roll)) - (sin(Yaw) * cos(Roll)));
    gsl_matrix_set(mat_out, 1, 1, (sin(Yaw) * sin(Pitch) * sin(Roll)) + (cos(Yaw) * cos(Roll)));
    gsl_matrix_set(mat_out, 1, 2, cos(Pitch) * sin(Roll));
    gsl_matrix_set(mat_out, 2, 0, (cos(Yaw) * sin(Pitch) * cos(Roll)) + (sin(Yaw) * sin(Roll)));
    gsl_matrix_set(mat_out, 2, 1, (sin(Yaw) * sin(Pitch) * cos(Roll)) - (cos(Yaw) * sin(Roll)));
    gsl_matrix_set(mat_out, 2, 2, cos(Pitch) * cos(Roll));

    return 0;
}

int sign(const double variable) {
    int sign = 0;
    if (variable < 0.)
        sign = -1;
    if (variable >= 0.)
        sign = 1;

    return sign;
}





