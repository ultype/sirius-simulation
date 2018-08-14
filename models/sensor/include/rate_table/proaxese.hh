#ifndef __PROAXESE_INTERFACE_HH__
#define __PROAXESE_INTERFACE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (ProAxeSE Rate Table interface definition)
*******************************************************************************/
#include <cstdint>

namespace sensor {

struct int32_xyz_t {
    int32_t x;
    int32_t y;
    int32_t z;
} __attribute__((packed));

struct ProAxeSE_data_t {
    int32_xyz_t    rate;            /* *io  (r/s)     X, Y, Z Rate */
} __attribute__((packed));

}  // namespace sensor

#endif  // __PROAXESE_INTERFACE__
