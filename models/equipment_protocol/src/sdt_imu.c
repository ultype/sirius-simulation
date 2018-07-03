#include "sdt_imu.h"
#include <string.h>

int imu_adis16488_receive(struct icf_ctrlblk_t *C, struct sdt_imu_dev_t *dev_info) {
    int rx_buff_size = sizeof(struct imu_adis16488_data_t);
    struct imu_adis16488_data_t *motion_data = &dev_info->raw_data;
    struct imu_motion_log_data_t *log_data = &dev_info->log_data;
    icf_rx_ctrl_job(C, HW_PORT1, rx_buff_size);
    if (icf_rx_dequeue(C, EGSE_IMU01_RX_SW_QIDX, (uint8_t *)motion_data, rx_buff_size) < 0) {
        fprintf(stderr, "[%s:%d] Received Error !! \n", __FUNCTION__, __LINE__);
        return -1;
    }
    return 0;
}
