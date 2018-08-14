#ifndef __EMU_GPS_HH__
#define __EMU_GPS_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (SimGen GPS Emulator interface definition)
*******************************************************************************/
#include <cstdint>

namespace sensor {

struct double_xyz_t {
    double x;
    double y;
    double z;
} __attribute__((packed));

struct MOT_t {
    double          timestamp;                      /* *io  (--)    Timestamp */

    double_xyz_t    position;                       /* *io  (m)     X, Y, Z position */
    double_xyz_t    velocity;                       /* *io  (m/s)   Velocity in X, Y, Z axes */
    double_xyz_t    acceleration;                   /* *io  (m/s2)  Acceleration in X, Y, Z axes */
    double_xyz_t    jerk;                           /* *io  (m/s3)  Jerk in X, Y, Z axes */

    double          heading;                        /* *io  (r)     Heading, range +/- PI */
    double          elevation;                      /* *io  (r)     Elevation, range +/- PI/2 */
    double          bank;                           /* *io  (r)     Bank, range +/- PI */

    double_xyz_t    angular_velocity;               /* *io  (r/s)   Angular velocity about X, Y, Z body axes */
    double_xyz_t    angular_acceleration;           /* *io  (r/s2)  Angular acceleration about X, Y, Z body axes */
    double_xyz_t    angular_jerk;                   /* *io  (r/s3)  Angular jerk about X, Y, Z body axes */
} __attribute__((packed));

}  // namespace sensor

#endif  // __EMU_GPS__
