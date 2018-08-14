#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Gyro model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux.hh>
#include "global_constants.hh"
#include "stochastic.hh"

namespace sensor {
class Gyro {
    TRICK_INTERFACE(sensor__Gyro);

 public:
    char name[256];

    Gyro() : VECTOR_INIT(EWBIB, 3), VECTOR_INIT(WBICB, 3), VECTOR_INIT(HIGH, 3), VECTOR_INIT(LOW, 3) {};

    virtual ~Gyro() {}

    virtual void propagate_error(double int_step) {}
    virtual void update_diagnostic_attributes(double int_step) {
            // decomposing computed body rates
            ppcx = get_ppcx();
            rrcx = get_rrcx();
            qqcx = get_qqcx();
    }

    virtual arma::vec3 get_computed_WBIB() { return WBICB; }
    virtual arma::vec3 get_error_of_computed_WBIB() { return EWBIB; }

    virtual double get_ppcx() { return WBICB(0) * DEG; }
    virtual double get_qqcx() { return WBICB(1) * DEG; }
    virtual double get_rrcx() { return WBICB(2) * DEG; }
    virtual arma::vec3 get_HIGH() { return HIGH; }
    virtual arma::vec3 get_LOW() { return LOW; }

    std::function<arma::vec3()> grab_WBIB;
    std::function<arma::vec3()> grab_FSPB;

 protected:
    arma::vec WBICB;     /* *o  (r/s)   Computed inertial body rate in body coordinate */
    double _WBICB[3];    /* *o  (r/s)   Computed inertial body rate in body coordinate */

    arma::vec EWBIB;     /* *o  (r/s)   Error in angular vel of body wrt earth */
    double _EWBIB[3];    /* *o  (r/s)   Error in angular vel of body wrt earth */

    arma::vec HIGH;
    double _HIGH[3];

    arma::vec LOW;
    double _LOW[3];

 private:
    double qqcx;        /* *o  (d/s)   INS computed pitch rate */
    double rrcx;        /* *o  (d/s)   INS computed yaw rate */
    double ppcx;        /* *o  (d/s)   INS computed roll rate */
};
}  // namespace sensor

#endif  // __GYRO__
