#ifndef __GPS_R_HH__
#define __GPS_R_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the GPS Receiver On Board)
LIBRARY DEPENDENCY:
      ((../src/GPS_receiver.cpp) (../src/utility_functions.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Rocket.hh"
#include "Ins.hh"
#include "GPS_satellites.hh"

class GPS_Receiver {
    public:
    GPS_Receiver() {}

    void default_data();
    void initialize(Rocket&, GPS_Satellites&, INS&);

    void get_quadriga();

    void

    Rocket *rocket;
    GPS_Satellites *gps_sats;
    INS *ins;

    bool gps_acq;            /* ** (--)       GPS Signal Acquired? */

    double del_rearth;       /* *i (m)        GPS Receiver LOS Minimum distance */

    double gdop;             /* *o (m)        Geometric dillution of precision of quadriga */
    double slot[4];          /* *o (--)       SV slot#  of quadriga */
    double ssii_quad[16];    /* *o (m)        Best quadriga inertial coordinates and their slot# */
    double vsii_quad[12];    /* *o (m/s)      Best quadriga inertial velocities */

};

#endif  // __GPS_R_HH__
