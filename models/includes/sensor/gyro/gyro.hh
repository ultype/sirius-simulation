#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Gyro model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

namespace sensor {
    class Gyro
    {
        TRICK_INTERFACE(sensor__Gyro);

        public:
            char name[256];

            Gyro() : VECTOR_INIT(EWBIB, 3), VECTOR_INIT(WBICB, 3) {};

            virtual ~Gyro(){};

            virtual void propagate_error(double int_step) {};
            virtual void update_diagnostic_attributes(double int_step) {};

            virtual arma::vec3 get_computed_WBIB() { return WBICB; };
            virtual arma::vec3 get_error_of_computed_WBIB() { return EWBIB; };

        protected:
            arma::vec WBICB;     /* *o  (r/s)   Computed inertial body rate in body coordinate */
            double _WBICB[3];    /* *o  (r/s)   Computed inertial body rate in body coordinate */

            arma::vec EWBIB;     /* *o  (r/s)   Error in angular vel of body wrt earth */
            double _EWBIB[3];    /* *o  (r/s)   Error in angular vel of body wrt earth */
    };
}

#endif//__GYRO__
