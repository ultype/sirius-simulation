#ifndef __SDT_INTERFACE_HH__
#define __SDT_INTERFACE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Sensor Data Transport (SDT) interface definition)
*******************************************************************************/
#include <cstdint>

#include "gps/nspo_gps.hh"
#include "imu/imu_interface.hh"

namespace sensor {

struct SDT_INTERFACER_t {
    NSPO_GPSR_SCI_TLM_t     GPS_data_1;  // 622 bytes
    NSPO_GPSR_SCI_TLM_t     GPS_data_2;  // 622 bytes
    IMU_filtered_data_t     IMU_filtered_data_1;  // 96 bytes
    IMU_filtered_data_t     IMU_filtered_data_2;  // 96 bytes
} __attribute__((packed));

}  // namespace sensor

#endif  // __SDT_INTERFACE__
