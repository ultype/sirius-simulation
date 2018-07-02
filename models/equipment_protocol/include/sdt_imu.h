#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_IMU_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_IMU_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      SDT GPSR
LIBRARY DEPENDENCY:
      (
        (../src/sdt_imu.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stdint.h>
#include "icf_trx_ctrl.h"
struct imu_adis16488_data_t {
    uint16_t accel_hi[3];
    uint16_t accel_low[3];
    uint16_t angular_vel_hi[3];
    uint16_t angular_vel_low[3];
} __attribute__((packed));

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declare */
int imu_adis16488_receive(struct icf_ctrlblk_t *C, void *data);
#ifdef __cplusplus
}
#endif

#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_IMU_H_
