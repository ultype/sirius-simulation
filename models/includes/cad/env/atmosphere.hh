#ifndef __Atmosphere_HH__
#define __Atmosphere_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Atmosphere model interface definition)
*******************************************************************************/

namespace cad {
    class Atmosphere
    {
        public:
            char name[256];

            Atmosphere() {};

            virtual ~Atmosphere() {};

            virtual void set_altitude(double altitude_in_meter) {};

            virtual double get_temperature_in_kelvin() { return tempk; };
            virtual double get_density() { return density; };
            virtual double get_pressure() { return pressure; };
            virtual double get_speed_of_sound() { return vsound; };

            virtual double get_speed_of_wind() { return vwind; };
            virtual double get_direction_of_wind() { return dwind; };

        protected:
            double altitude;

            double tempk;
            double density;
            double pressure;
            double vsound;

            double vwind;
            double dwind;
    };
}

#endif//__Atmosphere__
