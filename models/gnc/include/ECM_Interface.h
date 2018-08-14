#ifndef _ECM_INTERFACE_H_
#define _ECM_INTERFACE_H_

/* Data status code for data_ok */
#define ECM_DATA_OK 0
#define ECM_DATA_NOT_UPDATED 0x1
#define ECM_DATA_SDT_STATUS_ERR 0x2
#define ECM_DATA_DESYNC 0x4

/* GPSR part */
struct channel_status {
    uint8_t sv_id;
    uint8_t snr;
} __attribute__((packed));

typedef struct {
    uint8_t data_ok;
#ifdef DEBUG_ECM_FULL_GPSR_DATA
    uint8_t upd_counter;
    uint8_t sdt_status;
#endif
    int64_t timer_count;
    uint16_t gps_week;
    uint32_t gps_time;
    uint16_t validity;
    uint16_t pdop;
    float pos[3];
    float vel[3];
#ifdef DEBUG_ECM_FULL_GPSR_DATA
    int32_t oscill_offset;
    int64_t clock_bias;
    uint32_t health_sat_map;
#endif
    uint32_t visibility_sat_map;
#ifdef DEBUG_ECM_FULL_GPSR_DATA
    int16_t elevation_mask;
    uint8_t op_mode_it_flags;
    uint8_t raim_threshold;
#endif
    uint8_t c_no_threshold;
    struct channel_status channel[12];
    uint8_t boot_status;
#ifdef DEBUG_ECM_FULL_GPSR_DATA
    uint16_t delta_t_ls;
    uint16_t wn_lsf;
    uint8_t dn;
    uint8_t delta_t_lsf;
#endif
} __attribute__((packed)) ecm_gpsr_data_t;

/* IMU part */
struct gyro_accel_set {
    float gyro[3];
    float accel[3];
} __attribute__((packed));

typedef struct {
    int8_t data_ok;
    int64_t timer_count;
    float temp;
    struct gyro_accel_set latest;
    struct gyro_accel_set integral;
} __attribute__((packed)) ecm_imu_data_t;

/* The whole data */
typedef struct {
    ecm_gpsr_data_t gpsr[2];
    ecm_imu_data_t imu[2];
} __attribute__((packed)) ecm_to_gnc_t;

#endif  /* __ECM_INTERFACE_H__ */

