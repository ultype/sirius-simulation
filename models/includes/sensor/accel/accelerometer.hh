#ifndef __ACCEL_HH__
#define __ACCEL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Accelerometer model interface definition)
*******************************************************************************/
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <armadillo>
#include <aux/aux.hh>

namespace sensor {
    class Accelerometer
    {
        TRICK_INTERFACE(sensor__Accelerometer);

        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & _FSPCB;
                ar & _EFSPB;
            };

            char name[256];

            Accelerometer() : VECTOR_INIT(FSPCB, 3), VECTOR_INIT(EFSPB, 3) {};

            virtual ~Accelerometer(){};

            virtual void propagate_error(double int_step) {};
            virtual void update_diagnostic_attributes(double int_step) {};

            virtual arma::vec3 get_computed_FSPB() { return FSPCB; };
            virtual arma::vec3 get_error_of_computed_FSPB() { return EFSPB; };

        protected:
            arma::vec FSPCB;     /* *o  (N/kg)  Computed specific force on body */
            double _FSPCB[3];    /* *o  (N/kg)  Computed specific force on body */

            arma::vec EFSPB;     /* *o   (N/kg)  Error in specific force on body in body coordinate */
            double _EFSPB[3];    /* *o   (N/kg)  Error in specific force on body in body coordinate */
    };
}

#endif//__ACCEL__
