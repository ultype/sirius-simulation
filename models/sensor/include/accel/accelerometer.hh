#ifndef __ACCEL_HH__
#define __ACCEL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Accelerometer model interface definition)
*******************************************************************************/
#include <armadillo>
#include <aux.hh>
#include "global_constants.hh"
#include "icf_trx_ctrl.h"
#include "stochastic.hh"

namespace sensor {
class Accelerometer {
    TRICK_INTERFACE(sensor__Accelerometer);

 public:
    char name[256];

    Accelerometer() : VECTOR_INIT(FSPCB, 3), VECTOR_INIT(EFSPB, 3), VECTOR_INIT(HIGH, 3), VECTOR_INIT(LOW, 3) {};

    virtual ~Accelerometer() {}

    virtual void propagate_error(double int_step, struct icf_ctrlblk_t*) {}
    virtual void update_diagnostic_attributes(double int_step) {}

    std::function<arma::vec3()> grab_FSPB;

    virtual arma::vec3 get_computed_FSPB() { return FSPCB; }
    virtual arma::vec3 get_error_of_computed_FSPB() { return EFSPB; }
    virtual arma::vec3 get_HIGH() { return HIGH; }
    virtual arma::vec3 get_LOW() { return LOW; }

 protected:
    arma::vec FSPCB;     /* *o  (N/kg)  Computed specific force on body */
    double _FSPCB[3];    /* *o  (N/kg)  Computed specific force on body */

    arma::vec EFSPB;     /* *o   (N/kg)  Error in specific force on body in body coordinate */
    double _EFSPB[3];    /* *o   (N/kg)  Error in specific force on body in body coordinate */

    arma::vec HIGH;
    double _HIGH[3];

    arma::vec LOW;
    double _LOW[3];
};
}  // namespace sensor

#endif  // __ACCEL__
