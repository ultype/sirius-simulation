#include <cassert>
#include "aux.hh"

#include "math_utility.hh"
#include "integrate.hh"
#include "matrix/utility.hh"
#include "global_constants.hh"
#include <iostream>
#include "sdt/SDT_NONIDEAL.hh"

SDT_NONIDEAL::SDT_NONIDEAL()
    :   VECTOR_INIT(WBISB, 3),
        VECTOR_INIT(WBISB_old, 3),
        VECTOR_INIT(DELTA_ALPHA, 3),
        VECTOR_INIT(DELTA_ALPHA_old, 3),
        VECTOR_INIT(DELTA_BETA, 3),
        VECTOR_INIT(ALPHA, 3),
        VECTOR_INIT(BETA, 3),
        VECTOR_INIT(FSPSB, 3),
        VECTOR_INIT(FSPSB_old, 3),
        VECTOR_INIT(cross2_old, 3),
        VECTOR_INIT(cross3_old, 3),
        VECTOR_INIT(sculling_old, 3),
        VECTOR_INIT(VEL, 3),
        VECTOR_INIT(PHI_HIGH_OLD, 3),
        VECTOR_INIT(PHI_LOW_OLD, 3) {
            k = 0;
        }

// SDT_NONIDEAL::SDT_NONIDEAL(const SDT_NONIDEAL& other) {
//     SDT_NONIDEAL();

//     this->grab_WBICB = other.grab_WBICB;
//     this->grab_FSPCB = other.grab_FSPCB;
//     this->WBISB = other.WBISB;
//     this->WBISB_old = other.WBISB_old;
//     this->DELTA_ALPHA = other.DELTA_ALPHA;
//     this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
//     this->DELTA_BETA = other.DELTA_BETA;
//     this->ALPHA = other.ALPHA;
//     this->BETA = other.BETA;
//     this->FSPSB = other.FSPSB;
//     this->PHI = other.PHI;
// }

// SDT_NONIDEAL& SDT_NONIDEAL::operator=(const SDT_NONIDEAL& other) {
//     if (&other == this)
//         return *this;
//     this->grab_WBICB = other.grab_WBICB;
//     this->grab_FSPCB = other.grab_FSPCB;
//     this->WBISB = other.WBISB;
//     this->WBISB_old = other.WBISB_old;
//     this->DELTA_ALPHA = other.DELTA_ALPHA;
//     this->DELTA_ALPHA_old = other.DELTA_ALPHA_old;
//     this->DELTA_BETA = other.DELTA_BETA;
//     this->ALPHA = other.ALPHA;
//     this->BETA = other.BETA;
//     this->FSPSB = other.FSPSB;
//     this->PHI = other.PHI;
// }

void SDT_NONIDEAL::compute(double int_step) {
    WBISB = grab_WBICB();
    FSPSB = grab_FSPCB();
    if (k == 11) {
        k = 1;
        PHI.zeros();
        ALPHA.zeros();
        DELTA_VEL.zeros();
        VEL.zeros();
        PHI_HIGH.zeros();
        PHI_HIGH_OLD.zeros();
        PHI_LOW.zeros();
        PHI_LOW_OLD.zeros();
    }

    arma::vec3 GHIGH = grab_GHIGH();
    arma::vec3 GLOW = grab_GLOW();

    PHI_HIGH = PHI_HIGH_OLD + GHIGH;
    PHI_HIGH_OLD = PHI_HIGH;

    PHI_LOW = PHI_LOW_OLD + GLOW;
    PHI_LOW_OLD = PHI_LOW;
    // Coning compensation algorithm

    arma::vec3 zo(arma::fill::zeros);
    arma::vec3 CONING = grab_CONING();

    // arma::mat33 TBS = build_321_rotation_matrix(CONING);

    arma::vec3 tmp = WBISB + CONING;

    DELTA_ALPHA = integrate(tmp, WBISB_old, zo, int_step);
    // arma::vec3 cross1 = cross((ALPHA + DELTA_ALPHA), tmp);
    // arma::vec3 cross_old = cross(ALPHA, WBISB_old);

    // DELTA_BETA = integrate(cross1, cross_old, zo, int_step);  // cross((0.5 * (ALPHA + (1/6) * DELTA_ALPHA_old)), DELTA_ALPHA);  // integrate(cross1, cross_old, zo, int_step);

    ALPHA += DELTA_ALPHA;

    // BETA += DELTA_BETA;

    DELTA_ALPHA_old = DELTA_ALPHA;

    WBISB_old = tmp;

    PHI += DELTA_ALPHA;  // + 0.5 * DELTA_BETA;

    arma::vec3 cross2 = cross(ALPHA, FSPSB);
    // FSPSB = build_321_rotation_matrix(DELTA_ALPHA) * FSPSB;
    DELTA_VEL += integrate(FSPSB, FSPSB_old, zo, int_step) + integrate(cross2, cross2_old, zo, int_step);
    FSPSB_old = FSPSB;
    cross2_old = cross2;
    // arma::vec3 cross3 = cross(VEL, WBISB);
    // arma::vec3 sculling = 0.5 * (cross2 + cross3);
    // DELTA_VEL = VEL + 0.5 * cross(ALPHA, VEL);// + integrate(sculling, sculling_old, zo, int_step);
    // sculling_old = sculling;
    k++;
}

// arma::vec3 SDT_NONIDEAL::get_PHI() { return this->PHI; }
// arma::vec3 SDT_NONIDEAL::get_DELTA_VEL() { return this->DELTA_VEL; }
// arma::vec3 SDT_NONIDEAL::get_PHI_HIGH() { return this->PHI_HIGH; }
// arma::vec3 SDT_NONIDEAL::get_PHI_LOW() { return this->PHI_LOW; }

arma::mat33 SDT_NONIDEAL::build_321_rotation_matrix(arma::vec3 angle) {
    arma::mat33 TM;
    TM(0, 0) = cos(angle(2)) * cos(angle(1));
    TM(0, 1) = sin(angle(2)) * cos(angle(1));
    TM(0, 2) = -sin(angle(1));
    TM(1, 0) = (cos(angle(2)) * sin(angle(1)) * sin(angle(0))) - (sin(angle(2)) * cos(angle(0)));
    TM(1, 1) = (sin(angle(2)) * sin(angle(1)) * sin(angle(0))) + (cos(angle(2)) * cos(angle(0)));
    TM(1, 2) = cos(angle(1)) * sin(angle(0));
    TM(2, 0) = (cos(angle(2)) * sin(angle(1)) * cos(angle(0))) + (sin(angle(2)) * sin(angle(0)));
    TM(2, 1) = (sin(angle(2)) * sin(angle(1)) * cos(angle(0))) - (cos(angle(2)) * sin(angle(0)));
    TM(2, 2) = cos(angle(1)) * cos(angle(0));

    return TM;
}
