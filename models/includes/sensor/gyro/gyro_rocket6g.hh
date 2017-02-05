#ifndef __GYRO_ROCKET6G_HH__
#define __GYRO_ROCKET6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
LIBRARY DEPENDENCY:
      ((../../../src/sensor/gyro/gyro_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

#include "sensor/gyro/gyro.hh"

#include "rocket/Newton.hh"
#include "rocket/Euler.hh"
#include "rocket/Kinematics.hh"

namespace sensor {
    class GyroRocket6G : public Gyro
    {
        TRICK_INTERFACE(sensor__GyroRocket6G);

        public:
            char name[256];

            GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3], Newton &newt, _Euler_ &eul, Kinematics &kine);

            virtual ~GyroRocket6G() {};

            virtual void propagate_error(double int_step);

            virtual arma::vec3 get_computed_WBIB();
            virtual arma::vec3 get_error_of_WBIB();

        private:
            Newton     * newton;
            _Euler_    * euler;
            Kinematics * kinematics;

            arma::vec3 WBICB;    /* ** (--) */

            arma::vec EUG;       /* *o   (r/s)    Gyro spin axis accel sensitivity */
            double _EUG[3];      /* *o   (r/s)    Gyro spin axis accel sensitivity */

            arma::vec EWG;       /* *o   (r/s)    Gyro random walk errors */
            double _EWG[3];      /* *o   (r/s)    Gyro random walk errors */

            arma::vec EWBIB;     /* *o   (r/s)    Error in angular vel of body wrt earth */
            double _EWBIB[3];    /* *o   (r/s)    Error in angular vel of body wrt earth */

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

