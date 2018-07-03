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

struct imu_motion_log_data_t {
    double accel_log[3];  /* *io (m/s^2) accel */
    double angular_vel_log[3];  /* *io (°/s) angular velocity */
} __attribute__((packed));

struct sdt_imu_dev_t {
    struct imu_adis16488_data_t raw_data;
    struct imu_motion_log_data_t log_data;
};

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declare */
int imu_adis16488_receive(struct icf_ctrlblk_t *C, struct sdt_imu_dev_t *dev_info);
#ifdef __cplusplus
}
#endif

#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_IMU_H_
