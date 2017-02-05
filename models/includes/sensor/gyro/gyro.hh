#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

namespace sensor {
    class Gyro
    {
        TRICK_INTERFACE(sensor__Gyro);

        public:
            char name[256];

            Gyro() : VECTOR_INIT(RICI, 3), VECTOR_INIT(RICID, 3) {};

            virtual ~Gyro(){};

            virtual void propagate_error(double int_step) {};
            void update_diagnostic_attributes(double int_step) { ins_tilt_err = norm(RICI); };

            virtual arma::vec3 get_computed_WBIB() { return WBICB; };
            virtual arma::vec3 get_RICI() { return RICI; };

        protected:
            arma::vec3 WBICB;    /* *o  (r/s)   Computed inertial body rate in body coordinate */

            arma::vec RICI;      /* *o  (r)     INS tilt error derivative */
            double _RICI[3];     /* *o  (r)     INS tilt error */

            arma::vec RICID;     /* *o  (r)     INS tilt error derivative */
            double _RICID[3];    /* *o  (r)     INS tilt error derivative */

            double ins_tilt_err; /* *o  (r)     INS absolute tilt error */

    };
}

#endif//__GYRO__
