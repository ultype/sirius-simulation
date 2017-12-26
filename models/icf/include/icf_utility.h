#ifndef MODELS_ICF_INCLUDE_ICF_UTILITY_H_
#define MODELS_ICF_INCLUDE_ICF_UTILITY_H_
#include "icf_export.h"

#define CONFIG_ESPS_HEADER_ENABLE 1
#define CONFIG_EGSE_CRC_HEADER_ENABLE 1
#define DEBUG_ENABLE 1
/* Must can devide by 8*/
#define BILLION             1000000000L
#define FTRACE_TIME_STAMP(id) do { syscall(id);} while (0)

#define BIT(n) ((0x1U) << (n))
#define BITS(m, n) (~(BIT(m) - 1) & ((BIT(n) - 1) | BIT(n)))


#define SWAP32(x) \
    ((uint32_t) (\
    (((uint32_t) (x) & (uint32_t) 0x000000ffUL) << 24) | \
    (((uint32_t) (x) & (uint32_t) 0x0000ff00UL) << 8) | \
    (((uint32_t) (x) & (uint32_t) 0x00ff0000UL) >> 8) | \
    (((uint32_t) (x) & (uint32_t) 0xff000000UL) >> 24)))

#define cpu2le32(x) SWAP32((x))

#define errExit(msg)    do { perror(msg); \
                             while(1);} while (0)

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


#ifdef __cplusplus
extern "C" {
#endif
uint32_t crc32(uint32_t crc, const uint8_t *buf);
uint32_t crc32_create(const uint8_t *buf, const uint32_t len);
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
void debug_hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
uint32_t invert_crc32(uint32_t crc);
uint32_t crc_checker(uint32_t rx_crc, const uint8_t *buf, uint32_t size);
double get_curr_time(void);
int get_arr_num(int arrary_size, int element_size);
#ifdef __cplusplus
}
#endif

#endif   //  MODELS_ICF_INCLUDE_ICF_UTILITY_H_

