#ifndef __ACCEL_6G_HH__
#define __ACCEL_6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Accelerometer Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/accel/accelerometer_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux.hh>

#include "accel/accelerometer.hh"

namespace sensor {
class AccelerometerRocket6G : public Accelerometer {
    TRICK_INTERFACE(sensor__AccelerometerRocket6G);

 public:
    AccelerometerRocket6G(double emisa[3], double escala[3], double ebiasa[3]);

    virtual ~AccelerometerRocket6G() {}

    virtual void propagate_error(double int_step);

 private:
    arma::vec EWALKA;    /* *o   (m/s2)  Acceleration random noise */
    double _EWALKA[3];   /* *o   (m/s2)  Acceleration random noise */

    arma::vec EMISA;     /* *o   (r)     Acceleration misalignment */
    double _EMISA[3];    /* *o   (r)     Acceleration misalignment */

    arma::vec ESCALA;    /* *o   (--)    Acceleration scale factor */
    double _ESCALA[3];   /* *o   (--)    Acceleration scale factor */

    arma::vec EBIASA;    /* *o   (m/s2)  Acceleration bias */
    double _EBIASA[3];   /* *o   (m/s2)  Acceleration bias */

    arma::vec ITA1;
    double _ITA1[3];

    arma::vec ITA2;
    double _ITA2[3];

    arma::vec BETA;
    double _BETA[3];
};
}  // namespace sensor

#endif  // __ACCEL_6G__
