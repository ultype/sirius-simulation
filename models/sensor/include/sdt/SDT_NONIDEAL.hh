#ifndef __SDT_NONIDEAL_HH
#define __SDT_NONIDEAL_HH
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Sensor Data Transport module)
LIBRARY DEPENDENCY:
      ((../../src/SDT_nonideal.cpp))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "aux.hh"
#include "sdt/SDT.hh"

class SDT_NONIDEAL : public SDT{
    TRICK_INTERFACE(SDT_NONIDEAL);

 public:
    SDT_NONIDEAL();

    virtual void compute(double int_step);

 private:
    arma::mat33 build_321_rotation_matrix(arma::vec3 angle);


    arma::vec WBISB;        /* *o  (r/s)    Angular rate of body frame relative inertial frame as described in body frame sensed by gyro */
    double _WBISB[3];       /* *o  (r/s)    Angular rate of body frame relative inertial frame as described in body frame sensed by gyro */

    arma::vec WBISB_old;    /* *o  (r/s)    Angular rate of body frame relative inertial frame as described in body frame (previous time step) */
    double _WBISB_old[3];   /* *o  (r/s)    Angular rate of body frame relative inertial frame as described in body frame (previous time step) */

    arma::vec DELTA_ALPHA;  /* *o  (r)      Delta theta */
    double _DELTA_ALPHA[3]; /* *o  (r)      Delta theta */

    arma::vec DELTA_ALPHA_old;  /* *o (r)   Delta theta (previous time step) */
    double _DELTA_ALPHA_old[3]; /* *o (r)   Delta theta (previous time step) */

    arma::vec DELTA_BETA;   /* *o (r)       Delta beta */
    double _DELTA_BETA[3];  /* *o (r)       Delta beta */

    arma::vec ALPHA;        /* *o (r)       Alpha */
    double _ALPHA[3];       /* *o (r)       Alpha */

    arma::vec BETA;         /* *o (r)       Beta */
    double _BETA[3];        /* *o (r)       Beta */

    arma::vec FSPSB;        /* *o (m/s2)    Specific force of body frame sensed by accelerometer */
    double _FSPSB[3];       /* *o (m/s2)    Specific force of body frame sensed by accelerometer */

    arma::vec FSPSB_old;
    double _FSPSB_old[3];

    arma::vec cross2_old;
    double _cross2_old[3];

    arma::vec cross3_old;
    double _cross3_old[3];

    arma::vec sculling_old;
    double _sculling_old[3];

    arma::vec VEL;
    double _VEL[3];
    unsigned int k;         /* *o (--)      calculate index */

    arma::vec PHI_HIGH_OLD;
    double _PHI_HIGH_OLD[3];

    arma::vec PHI_LOW_OLD;
    double _PHI_LOW_OLD[3];
};


#endif
