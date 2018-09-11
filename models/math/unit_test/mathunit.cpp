#include "math_utility_c.h"
#include "matrix/utility.hh"
#include "global_constants.hh"
#include <iostream>
#include <cstdio>

int Vector_Error(gsl_vector *vec_gsl, arma::vec3 vec_arma);
int Matrix_Error(gsl_matrix *mat_gsl, arma::mat33 mat_arma);
int Quaternion_Error(gsl_vector *vec_gsl, arma::vec4 vec_arma);
int pol_from_cart_test();
int skew_sym_test();
int build_psivg_thtvg_TM_test();
int build_psi_tht_phi_TM_test();
int Matrix2Quaternion_test();
int Quaternion2Matrix_test();
int Quaternion_conjugate_test();
int Quaternion_cross_test();
int Quaternion2Euler_test();
int Euler2Quaternion_test();
int QuaternionMultiply_test();
int QuaternionInverse_test();
void print_matrix(const gsl_matrix *m);
void print_vector (const gsl_vector * v);
int moore_penrose_pinv_test();

int main(int argc, char const *argv[]) {
    fprintf(stderr, "** Math function unit test **\n");
    pol_from_cart_test();
    skew_sym_test();
    build_psivg_thtvg_TM_test();
    build_psi_tht_phi_TM_test();
    Matrix2Quaternion_test();
    Quaternion2Matrix_test();
    Quaternion_conjugate_test();
    Quaternion_cross_test();
    Quaternion2Euler_test();
    Euler2Quaternion_test();
    QuaternionInverse_test();
    moore_penrose_pinv_test();

    return 0;
}

void print_matrix(const gsl_matrix *m) {
    size_t i, j;

    for (i = 0; i < m->size1; i++) {
        for (j = 0; j < m->size2; j++) {
            printf("%f\t", gsl_matrix_get(m, i, j));
        }
        printf("\n");
    }
}

void print_vector (const gsl_vector * v) {
    size_t i;

    for (i = 0; i < v->size; i++) {
        printf("%f\t", gsl_vector_get (v, i));
    }
}

int Vector_Error(gsl_vector *vec_gsl, arma::vec3 vec_arma) {
    fprintf(stderr, "Error = [%.16f\t%.16f\t%.16f]\n", vec_arma(0) - gsl_vector_get(vec_gsl, 0), vec_arma(1) - gsl_vector_get(vec_gsl, 1), vec_arma(2) - gsl_vector_get(vec_gsl, 2));

    return 0;
}

int Quaternion_Error(gsl_vector *vec_gsl, arma::vec4 vec_arma) {
    fprintf(stderr, "Error = [%.16f\t%.16f\t%.16f\t%.16f]\n", vec_arma(0) - gsl_vector_get(vec_gsl, 0), vec_arma(1) - gsl_vector_get(vec_gsl, 1), vec_arma(2) - gsl_vector_get(vec_gsl, 2), vec_arma(3) - gsl_vector_get(vec_gsl, 3));

    return 0;
}

int Matrix_Error(gsl_matrix *mat_gsl, arma::mat33 mat_arma) {
    fprintf(stderr, "Error = \n");
    for (int i = 0; i < 3; i++) {
        fprintf(stderr, "|%.16f\t%.16f\t%.16f|\n", mat_arma(i, 0) - gsl_matrix_get(mat_gsl, i, 0), mat_arma(i, 1) - gsl_matrix_get(mat_gsl, i, 1), mat_arma(i, 2) - gsl_matrix_get(mat_gsl, i, 2));
    }
    return 0;
}

int pol_from_cart_test() {
    gsl_vector *cart;
    gsl_vector *pol;
    cart = gsl_vector_calloc(3);
    pol = gsl_vector_calloc(3);

    gsl_vector_set(cart , 0, 1.0);
    gsl_vector_set(cart , 1, 2.0);
    gsl_vector_set(cart , 2, 3.0);

    fprintf(stderr, "pol_from_cart_C test :\n");
    pol_from_cart_C(cart, pol);

    arma::vec3 cartt, poll;

    cartt(0) = 1.0;
    cartt(1) = 2.0;
    cartt(2) = 3.0;

    poll = pol_from_cart(cartt);

    Vector_Error(pol, poll);

    gsl_vector_free(cart);
    gsl_vector_free(pol);

    return 0;
}

int skew_sym_test() {
    gsl_matrix *skew_sym_c;
    gsl_vector *vec_in_c;

    skew_sym_c = gsl_matrix_calloc(3, 3);
    vec_in_c = gsl_vector_calloc(3);

    arma::vec3 vec_in;
    arma::mat33 skew_sym_matrix;

    for (int i = 0; i < 3; i++) {
        gsl_vector_set(vec_in_c, i, static_cast<double>(i));
        vec_in(i) = i;
    }

    skew_sym_c = skew_sym_C(vec_in_c);
    skew_sym_matrix = skew_sym(vec_in);
    fprintf(stderr, "skew_sym_C test :\n");
    Matrix_Error(skew_sym_c, skew_sym_matrix);

    gsl_vector_free(vec_in_c);
    gsl_matrix_free(skew_sym_c);
    return 0;
}

int build_psivg_thtvg_TM_test() {
    gsl_matrix *TM_gsl;
    TM_gsl = gsl_matrix_calloc(3, 3);

    arma::mat33 TM_arma;

    build_psivg_thtvg_TM_C(1.0, 2.0, TM_gsl);
    TM_arma = build_psivg_thtvg_TM(1.0, 2.0);

    fprintf(stderr, "build_psivg_thtvg_TM_C test :\n");

    Matrix_Error(TM_gsl, TM_arma);

    gsl_matrix_free(TM_gsl);

    return 0;
}


int build_psi_tht_phi_TM_test() {
    gsl_matrix *TM_gsl;
    arma::mat33 TM_arma;

    TM_gsl = gsl_matrix_calloc(3, 3);

    build_psi_tht_phi_TM_C(85.0 * __RAD, 75.0 * __RAD, 0.0, TM_gsl);
    TM_arma = build_psi_tht_phi_TM(85.0 * __RAD, 75.0 * __RAD, 0.0);

    fprintf(stderr, "build_psi_tht_phi_TM_C test :\n");
    Matrix_Error(TM_gsl, TM_arma);

    gsl_matrix_free(TM_gsl);
    return 0;
}

int Matrix2Quaternion_test() {
    gsl_matrix *Matrix_gsl;
    gsl_vector *Vector_gsl;

    Matrix_gsl = gsl_matrix_calloc(3, 3);
    Vector_gsl = gsl_vector_calloc(4);

    arma::mat33 Matrix_arma;
    arma::vec4 Vector_arma;

    build_psi_tht_phi_TM_C(90.0 * __RAD, 90 * __RAD, 0.0, Matrix_gsl);
    Matrix_arma = build_psi_tht_phi_TM(90.0 * RAD, 90.0 * RAD, 0.0);

    Matrix2Quaternion_C(Matrix_gsl, Vector_gsl);
    Vector_arma = Matrix2Quaternion(Matrix_arma);

    fprintf(stderr, "Matrix2Quaternion_C test : \n");
    Quaternion_Error(Vector_gsl, Vector_arma);

    gsl_matrix_free(Matrix_gsl);
    gsl_vector_free(Vector_gsl);

    return 0;
}

int Quaternion2Matrix_test() {
    gsl_vector *Vector_gsl;
    gsl_matrix *Matrix_gsl;

    Vector_gsl = gsl_vector_calloc(4);
    Matrix_gsl = gsl_matrix_calloc(3, 3);

    arma::vec4 Vector_arma;
    arma::mat33 Matrix_arma;

    for (int i = 0; i < 4; i++) {
        gsl_vector_set(Vector_gsl, i, static_cast<double>(i));
        Vector_arma(i) = static_cast<double>(i);
    }

    Quaternion2Matrix_C(Vector_gsl, Matrix_gsl);
    Matrix_arma = Quaternion2Matrix(Vector_arma);

    fprintf(stderr, "Quaternion2Matrix_C test : \n");

    Matrix_Error(Matrix_gsl, Matrix_arma);

    gsl_vector_free(Vector_gsl);
    gsl_matrix_free(Matrix_gsl);

    return 0;
}

int Quaternion_conjugate_test() {
    gsl_vector *Vector_gsl;
    Vector_gsl = gsl_vector_calloc(4);


    arma::vec4 Vector_arma;

    for (int i = 0; i < 4; i++) {
        gsl_vector_set(Vector_gsl, i, (double)i);
        Vector_arma(i) = (double)i;
    }

    Vector_gsl = Quaternion_conjugate_C(Vector_gsl);
    Vector_arma = Quaternion_conjugate(Vector_arma);

    fprintf(stderr, "Quaternion_conjugate_C test : \n");
    Quaternion_Error(Vector_gsl, Vector_arma);

    gsl_vector_free(Vector_gsl);
    return 0;
}

int Quaternion_cross_test() {
    gsl_vector *Vector_in1_gsl;
    gsl_vector *Vector_in2_gsl;
    gsl_vector *Vector_out_gsl;

    Vector_in1_gsl = gsl_vector_calloc(4);
    Vector_in2_gsl = gsl_vector_calloc(4);
    Vector_out_gsl = gsl_vector_calloc(4);

    arma::vec4 Vector_in1_arma;
    arma::vec4 Vector_in2_arma;
    arma::vec4 Vector_out_arma;

    for (int i = 0; i < 4; i++) {
        gsl_vector_set(Vector_in1_gsl, i, static_cast<double>(i));
        gsl_vector_set(Vector_in2_gsl, i, static_cast<double>(i));

        Vector_in1_arma(i) = static_cast<double>(i);
        Vector_in2_arma(i) = static_cast<double>(i);
    }

    Quaternion_cross_C(Vector_in1_gsl, Vector_in2_gsl, Vector_out_gsl);
    Vector_out_arma = Quaternion_cross(Vector_in1_arma, Vector_in2_arma);

    fprintf(stderr, "Quaternion_cross_C test : \n");

    Quaternion_Error(Vector_out_gsl, Vector_out_arma);

    gsl_vector_free(Vector_in1_gsl);
    gsl_vector_free(Vector_in2_gsl);
    gsl_vector_free(Vector_out_gsl);

    return 0;
}

int Quaternion2Euler_test() {
    gsl_vector *Vector_gsl;
    Vector_gsl = gsl_vector_calloc(4);

    arma::vec4 Vector_arma;

    double Roll_gsl, Pitch_gsl, Yaw_gsl;
    double Roll_arma, Pitch_arma, Yaw_arma;

    gsl_vector_set(Vector_gsl, 0, 0.5);
    gsl_vector_set(Vector_gsl, 1, -0.5);
    gsl_vector_set(Vector_gsl, 2, 0.5);
    gsl_vector_set(Vector_gsl, 3, 0.5);
    Vector_arma(0) = 0.5;
    Vector_arma(1) = -0.5;
    Vector_arma(2) = 0.5;
    Vector_arma(3) = 0.5;


    Quaternion2Euler_C(Vector_gsl, &Roll_gsl, &Pitch_gsl, &Yaw_gsl);
    Quaternion2Euler(Vector_arma, Roll_arma, Pitch_arma, Yaw_arma);

    fprintf(stderr, "Quaternion2Euler_C test : \n");

    fprintf(stderr, "Error = [%.14f\t%.14f\t%.14f]\n", Roll_gsl - Roll_arma, Pitch_gsl - Pitch_arma, Yaw_gsl - Yaw_arma);

    gsl_vector_free(Vector_gsl);

    return 0;
}

int Euler2Quaternion_test() {
    gsl_vector * Vector_gsl;
    Vector_gsl = gsl_vector_calloc(4);

    arma::vec4 Vector_arma;

    Euler2Quaternion_C(0.0, 90.0 * __RAD, 90.0 * __RAD, Vector_gsl);
    Vector_arma = Euler2Quaternion(0.0, 90.0 * __RAD, 90.0 * __RAD);

    fprintf(stderr, "Euler2Quaternion_C test : \n");
    Quaternion_Error(Vector_gsl, Vector_arma);

    gsl_vector_free(Vector_gsl);

    return 0;
}

int QuaternionInverse_test() {
    gsl_vector *Vector_in_gsl, *Vector_out_gsl;

    Vector_in_gsl = gsl_vector_calloc(4);
    Vector_out_gsl = gsl_vector_calloc(4);

    arma::vec4 Vector_in_arma, Vector_out_arma;

    gsl_vector_set(Vector_in_gsl, 0, 0.5);
    gsl_vector_set(Vector_in_gsl, 1, -0.5);
    gsl_vector_set(Vector_in_gsl, 2, 0.5);
    gsl_vector_set(Vector_in_gsl, 3, 0.5);

    Vector_in_arma(0) = 0.5;
    Vector_in_arma(1) = -0.5;
    Vector_in_arma(2) = 0.5;
    Vector_in_arma(3) = 0.5;

    Vector_out_gsl = QuaternionInverse_C(Vector_in_gsl);
    Vector_out_arma = QuaternionInverse(Vector_in_arma);

    fprintf(stderr, "QuaternionInverse_C test : \n");
    Quaternion_Error(Vector_out_gsl, Vector_out_arma);

    return 0;
}

int moore_penrose_pinv_test() {
    const unsigned int N = 2;
    const unsigned int M = 3;
    const double rcond = 1E-15;


    gsl_matrix *A = gsl_matrix_alloc(N, M);
    gsl_matrix *A_pinv;

    gsl_matrix_set(A, 0, 0, 1.);
    gsl_matrix_set(A, 0, 1, 3.);
    gsl_matrix_set(A, 0, 2, 5.);
    gsl_matrix_set(A, 1, 0, 2.);
    gsl_matrix_set(A, 1, 1, 4.);
    gsl_matrix_set(A, 1, 2, 6.);

    printf("A matrix:\n");
    print_matrix(A);
    A_pinv = moore_penrose_pinv(A, rcond);
    printf("\nPseudoinverse of A:\n");
    print_matrix(A_pinv);

    gsl_matrix_free(A);
    gsl_matrix_free(A_pinv);

    return 0;
}














