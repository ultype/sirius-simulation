#include "sdt_imu.h"
#include <string.h>

int imu_adis16488_init(struct sdt_imu_dev_t *dev_info) {
    memset(dev_info, 0, sizeof(struct sdt_imu_dev_t));
    return 0;
}

int imu_adis16488_receive(struct icf_ctrlblk_t *C, struct sdt_imu_dev_t *dev_info) {
    int rx_buff_size = sizeof(struct imu_adis16488_data_t);
    struct imu_adis16488_data_t *motion_data = &dev_info->raw_data;
    struct imu_motion_log_data_t *log_data = &dev_info->log_data;
    icf_rx_ctrl_job(C, HW_PORT1, rx_buff_size);
    if (icf_rx_dequeue(C, EGSE_IMU01_RX_SW_QIDX, (uint8_t *)motion_data, rx_buff_size) < 0) {
        fprintf(stderr, "[%s:%d] Received Error !! \n", __FUNCTION__, __LINE__);
        return -1;
    }

    log_data->accel_log[0] = ((int16_t)motion_data->accel_hi[0] + motion_data->accel_low[0] / 65536.0) * ADIS16488_ACCEL_RES * LOCAL_GRAVITY;
    log_data->accel_log[1] = ((int16_t)motion_data->accel_hi[1] + motion_data->accel_low[1] / 65536.0) * ADIS16488_ACCEL_RES * LOCAL_GRAVITY;
    log_data->accel_log[2] = ((int16_t)motion_data->accel_hi[2] + motion_data->accel_low[2] / 65536.0) * ADIS16488_ACCEL_RES * LOCAL_GRAVITY;
    log_data->angular_vel_log[0] = ((int16_t)motion_data->angular_vel_hi[0] + motion_data->angular_vel_low[0] / 65536.0) * ADIS16488_GYRO_RES;
    log_data->angular_vel_log[1] = ((int16_t)motion_data->angular_vel_hi[1] + motion_data->angular_vel_low[1] / 65536.0) * ADIS16488_GYRO_RES;
    log_data->angular_vel_log[2] = ((int16_t)motion_data->angular_vel_hi[2] + motion_data->angular_vel_low[2] / 65536.0) * ADIS16488_GYRO_RES;

    return 0;
}
