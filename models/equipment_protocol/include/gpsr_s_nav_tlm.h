#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_GPSR_S_NAV_TLM_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_GPSR_S_NAV_TLM_H_
#include <stdint.h>

#define SV_CHANNEL_NUM 12

struct sv_channel_t {
    uint8_t sv_id;
    uint8_t snr;
} __attribute__ ((packed));

struct gpsr_s_nav_tlm_frame_t {
    uint16_t gps_week_num;
    uint32_t gps_time;
    uint16_t validity;
    uint16_t pdop;
    float posx;
    float posy;
    float posz;
    float velx;
    float vely;
    float velz;
    int32_t oscill_offset;
    int64_t clock_bias;
    uint32_t health_satellites_map;
    uint32_t visibility_satellites_map;
    int16_t elevation_mask;
    union {
        uint8_t packed;
        struct {
            uint8_t op_mode:1;
            uint8_t ionosphere:2;
            uint8_t troposphere:1;
        };
    } op_ionosphere_troposphere_;
    uint8_t raim_threshold;
    uint8_t c_no_tracking_threshold;
    struct sv_channel_t sv_channels[SV_CHANNEL_NUM];
    union {
        uint8_t packed;
        struct {
            uint8_t boot_config:1;
            uint8_t selected_image:1;
            uint8_t loaded_image:1;
            uint8_t reserved:1;
            uint8_t boot_config_status:2;
            uint8_t selected_image_status:2;
        };
    } boot_status_;
    uint16_t delta_t_ls;
    uint16_t wn_lsf;
    uint8_t dn;
    uint8_t delta_t_lsf;
} __attribute__ ((packed));
#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_GPSR_S_NAV_TLM_H_