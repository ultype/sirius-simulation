#ifndef __wind_HH__
#define __wind_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
LIBRARY DEPENDENCY:
      ((../../../src/cad/env/wind.cpp))
*******************************************************************************/

#include <armadillo>
#include <aux/aux.hh>

namespace cad {
    class Wind
    {
        TRICK_INTERFACE(cad__Wind);

        public:
            char name[256];

            Wind(double twind, double vertical_wind);

            virtual ~Wind() {};

            virtual void set_altitude(double altitude_in_meter) {};
            virtual void propagate_VAED(double int_step);

            virtual double get_speed_of_wind() { return vwind; };
            virtual double get_direction_of_wind() { return psiwdx; };

            virtual arma::vec3 get_VAED() { return VAED; };
            virtual arma::vec3 get_VAEDS() { return VAEDS; };
            virtual arma::vec3 get_VAEDSD() { return VAEDSD; };

        protected:
            double twind;
            double altitude;

            double vertical_wind_speed;

            double vwind;
            double psiwdx;      /* *io (m/s)        Wind direction from north - m/s*/

        private:
            arma::vec VAED;     /* *o (m/s)        Smoothed wind velocity in geodetic coord */
            double _VAED[3];    /* *o (m/s)        Smoothed wind velocity in geodetic coord */

            arma::vec VAEDS;    /* *o (m/s)        Smoothed wind velocity in geodetic coord - m/s */
            double _VAEDS[3];   /* *o (m/s)        Smoothed wind velocity in geodetic coord - m/s */

            arma::vec VAEDSD;   /* *o (m/s)        Smoothed wind velocity derivative - m/s */
            double _VAEDSD[3];  /* *o (m/s)        Smoothed wind velocity derivative - m/s */
    };
}

#endif//__wind__
