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

#include "Newton.hh"
#include "Euler.hh"
#include "Kinematics.hh"

#include <boost/archive/text_oarchive.hpp>

class Kinematics;
class Newton;
class _Euler_;

namespace sensor {
    class GyroRocket6G : public Gyro
    {
        TRICK_INTERFACE(sensor__GyroRocket6G);

        public:
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version){
                ar & boost::serialization::base_object<Gyro>(*this);

                ar & newton;
                ar & euler;
                ar & kinematics;

                ar & _EUG;
                ar & _EWG;
                ar & _EWALKG;
                ar & _EUNBG;
                ar & _EMISG;
                ar & _ESCALG;
                ar & _EBIASG;
            };

            GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3], Newton &newt, _Euler_ &eul, Kinematics &kine);

            virtual ~GyroRocket6G() {};

            virtual void propagate_error(double int_step);

        private:
            /* Routing components */
            Newton     * newton;
            _Euler_    * euler;
            Kinematics * kinematics;

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
    };
}

#endif//__GYRO_ROCKET6G__

