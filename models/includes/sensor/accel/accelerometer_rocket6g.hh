#ifndef __ACCEL_6G_HH__
#define __ACCEL_6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Accelerometer Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../../src/sensor/accel/accelerometer_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

#include "sensor/accel/accelerometer.hh"
#include "rocket/Newton.hh"

namespace sensor {
    class AccelerometerRocket6G : public Accelerometer
    {
        TRICK_INTERFACE(sensor__AccelerometerRocket6G);

        public:

            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Accelerometer>(*this);

                ar & _EWALKA;
                ar & _EMISA;
                ar & _ESCALA;
                ar & _EBIASA;
            }

            AccelerometerRocket6G(double emisa[3], double escala[3], double ebiasa[3], Newton &newt);

            virtual ~AccelerometerRocket6G() {};

            virtual void propagate_error(double int_step);

        private:
            Newton * newton;

            arma::vec EWALKA;    /* *o   (m/s2)  Acceleration random noise */
            double _EWALKA[3];   /* *o   (m/s2)  Acceleration random noise */

            arma::vec EMISA;     /* *o   (r)     Acceleration misalignment */
            double _EMISA[3];    /* *o   (r)     Acceleration misalignment */

            arma::vec ESCALA;    /* *o   (--)    Acceleration scale factor */
            double _ESCALA[3];   /* *o   (--)    Acceleration scale factor */

            arma::vec EBIASA;    /* *o   (m/s2)  Acceleration bias */
            double _EBIASA[3];   /* *o   (m/s2)  Acceleration bias */
    };
}

#endif//__ACCEL_6G__
