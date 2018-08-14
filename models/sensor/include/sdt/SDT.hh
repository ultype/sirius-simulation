#ifndef __SDT_HH
#define __SDT_HH
#include <armadillo>
#include <functional>
#include "aux.hh"

class SDT {
    TRICK_INTERFACE(SDT);

 public:
    SDT() :         VECTOR_INIT(PHI, 3), VECTOR_INIT(DELTA_VEL, 3), VECTOR_INIT(PHI_HIGH, 3), VECTOR_INIT(PHI_LOW, 3) {};
    virtual ~SDT() {};

    virtual void compute(double int_step) {}

    std::function<arma::vec3()> grab_WBICB;
    std::function<arma::vec3()> grab_FSPCB;
    std::function<arma::vec3()> grab_CONING;
    std::function<arma::vec3()> grab_GHIGH;
    std::function<arma::vec3()> grab_GLOW;
    std::function<arma::vec3()> grab_AHIGH;
    std::function<arma::vec3()> grab_ALOW;

    virtual arma::vec3 get_PHI() { return PHI; }
    virtual arma::vec3 get_DELTA_VEL() { return DELTA_VEL; }
    virtual arma::vec3 get_PHI_HIGH() { return PHI_HIGH; }
    virtual arma::vec3 get_PHI_LOW() { return PHI_LOW; }

 protected:
    virtual arma::mat33 build_321_rotation_matrix(arma::vec3 angle) {}

    arma::vec PHI;          /* *o (r)       PHI = DELTA_ALPHA + 0.5 * DELTA_BETA */
    double _PHI[3];          /* *o (r)       PHI = DELTA_ALPHA + 0.5 * DELTA_BETA */

    arma::vec DELTA_VEL;    /* *o (m/s)     Delta V */
    double _DELTA_VEL[3];   /* *o (m/s)     Delta V */

    arma::vec PHI_HIGH;
    double _PHI_HIGH[3];

    arma::vec PHI_LOW;
    double _PHI_LOW[3];
};

#endif
