#ifndef __GYRO_ROCKET6G_HH__
#define __GYRO_ROCKET6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Gyro Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "gyro/gyro.hh"

namespace sensor {
class GyroRocket6G : public Gyro {
    TRICK_INTERFACE(sensor__GyroRocket6G);

 public:
    GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3]);

    virtual ~GyroRocket6G() {}

    virtual void propagate_error(double int_step);
    std::default_random_engine generator;

 private:
    /* Routing components */
    arma::vec EUG;       /* *o   (r/s)    Gyro spin axis accel sensitivity */
    double _EUG[3];      /* *o   (r/s)    Gyro spin axis accel sensitivity */

    arma::vec EWG;       /* *o   (r/s)    Gyro random walk errors */
    double _EWG[3];      /* *o   (r/s)    Gyro random walk errors */

    arma::vec EWALKG;    /* *o   (r/s0.5) Random walk */
    double _EWALKG[3];   /* *o   (r/s0.5) Random walk */

    arma::vec EUNBG;     /* *o   (r)      Gyro cluster misalignment */
    double _EUNBG[3];    /* *o   (r)      Gyro cluster misalignment */

    arma::vec EMISG;     /* *o   (r)      Gyro misalignmt */
    double _EMISG[3];    /* *o   (r)      Gyro misalignmt */

    arma::vec ESCALG;    /* *o   (--)     Gyro scale factor */
    double _ESCALG[3];   /* *o   (--)     Gyro scale factor */

    arma::vec EBIASG;    /* *o   (r/s)    Gyro bias */
    double _EBIASG[3];   /* *o   (r/s)    Gyro bias */

    arma::vec ITA1;
    double _ITA1[3];

    arma::vec ITA2;
    double _ITA2[3];

    arma::vec BETA;
    double _BETA[3];
};
}  // namespace sensor

#endif  // __GYRO_ROCKET6G__

