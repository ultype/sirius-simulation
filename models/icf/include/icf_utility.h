#ifndef MODELS_ICF_INCLUDE_ICF_UTILITY_H_
#define MODELS_ICF_INCLUDE_ICF_UTILITY_H_
#include "icf_export.h"
#include "nspo_gps.h"
#include "imu_interface.h"

#define CONFIG_ESPS_HEADER_ENABLE 1
#define CONFIG_EGSE_CRC_HEADER_ENABLE 1
/* Must can devide by 8*/
#define BILLION             1000000000L
#define FTRACE_TIME_STAMP(id) do { syscall(id);} while (0)

#define SWAP32(x) \
    ((uint32_t) (\
    (((uint32_t) (x) & (uint32_t) 0x000000ffUL) << 24) | \
    (((uint32_t) (x) & (uint32_t) 0x0000ff00UL) << 8) | \
    (((uint32_t) (x) & (uint32_t) 0x00ff0000UL) >> 8) | \
    (((uint32_t) (x) & (uint32_t) 0xff000000UL) >> 24)))

#define cpu2le32(x) SWAP32((x))

#define errExit(msg)    do { perror(msg); \
                             exit(EXIT_FAILURE);} while (0)
#define DEBUG_ENABLE 0
#define debug_print(...) do { if (DEBUG_ENABLE) \
                                   fprintf(stderr, __VA_ARGS__);} while (0)

/**
 * @brief   TiSPACE EGSE System Profiling Server (ESPS)
 */

#define TVC_SIZE 6
#define RCS_SIZE 4
#define II_VALUE_CONTROL_SIZE 3
#define ORDANCE_SIZE 3

struct esps2egse_header_t {
    uint32_t payload_len;
    uint32_t crc;
} __attribute__((packed));
#define ESPS2EGSE_HEADER_SIZE (sizeof(struct esps2egse_header_t))

/*
*
* EGSE->DM 20pps, RX cmd format from devices. (egse data verify server)
*
*/
struct esps2egse_data_t {
#if CONFIG_ESPS_HEADER_ENABLE
    uint8_t III_TVC_1[TVC_SIZE];
    uint8_t III_TVC_2[TVC_SIZE];
    uint8_t III_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t III_valve_control_2[II_VALUE_CONTROL_SIZE];
    uint8_t RCS[RCS_SIZE];
    uint8_t ordance_faring[ORDANCE_SIZE];
    uint8_t ordance_separation[ORDANCE_SIZE];
    uint8_t II_TVC_1[TVC_SIZE];
    uint8_t II_TVC_2[TVC_SIZE];
    uint8_t II_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t II_valve_control_2[II_VALUE_CONTROL_SIZE];
    uint8_t reserve[2];
#else
    uint8_t single_cmd[8];
#endif
} __attribute__((packed));



/*
*
* /dev/ttyAP0, /dev/ttyAP4: SDT_INTERFACER_t 200HZ for IMU
* /dev/ttyAP5, /dev/ttyAP6: SDT_INTERFACER_t 20HZ for GPSR
*
*/
struct SDT_INTERFACER_t {
    struct NSPO_GPSR_SCI_TLM_t     GPS_data_1;  // 624 bytes
    struct NSPO_GPSR_SCI_TLM_t     GPS_data_2;  // 624 bytes (redundancy)
    struct IMU_filtered_data_t     IMU_filtered_data_1;  // 96 bytes
    struct IMU_filtered_data_t     IMU_filtered_data_2;  // 96 bytes (redundancy)
} __attribute__((packed));

/*
*
* /dev/ttyAP1(x), /dev/ttyAP2(y), /dev/ttyAP3(z): ProAxeSE_data_t 1000HZ
*
*/
struct int32_xyz_t {
    int32_t x;
    int32_t y;
    int32_t z;
} __attribute__((packed));

struct ProAxeSE_data_t {
    struct int32_xyz_t    rate;            /* *io  (r/s)     X, Y, Z Rate */
} __attribute__((packed));


#ifdef __cplusplus
extern "C" {
#endif
uint32_t crc32(uint32_t crc, const uint8_t *buf);
uint32_t crc32_create(const uint8_t *buf, const uint32_t len);
int32_t imu_pattern_init(struct IMU_filtered_data_t *imu);
int32_t rate_table_pattern_init(struct ProAxeSE_data_t *position);
int32_t gpsr_pattern_init(void *gpsr_data);
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
void debug_hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
uint32_t invert_crc32(uint32_t crc);
uint32_t crc_checker(uint32_t rx_crc, const uint8_t *buf, uint32_t size);
double get_curr_time(void);
#ifdef __cplusplus
}
#endif

#endif   //  MODELS_ICF_INCLUDE_ICF_UTILITY_H_

