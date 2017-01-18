#ifndef __GPS_SAT_HH__
#define __GPS_SAT_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the 24 GPS satellites in orbit)
LIBRARY DEPENDENCY:
      ((../src/GPS_satellites.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

class GPS_Satellites {
    public:
        GPS_Satellites() {}

        void default_data();
        void initialize();

        void propagate_sv();

        double sv_data[24][2];   /* *i (--)       Inputed GPS sv position */
        double inclination;      /* ** (r)        GPS Satellite Orbit inclination*/
        double radius;           /* ** (m)        GPS Satellite Orbit radius */
        double time;             /* ** (s)        Time of last propagation */
        double angular_velocity; /* ** (m/s)      GPS Satellite Orbit velocity */
    private:
        double almanac_time;     /* *i (s)        Time delta between the almanac and start of Simulation */
        //XXX: Should specify units
        double sv_init_data[48]; /* ** (--)       GPS sv position when init */

};

#endif  // __GPS_SAT_HH__
