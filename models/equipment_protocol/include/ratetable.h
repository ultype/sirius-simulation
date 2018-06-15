#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_RATETABLE_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_RATETABLE_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      SDT GPSR
LIBRARY DEPENDENCY:
      (
        (../src/ratetable.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "icf_trx_ctrl.h"

typedef enum _RATETABLE_MOVEMENT_MODE_ENUM {
    RATETABLE_MOVEMENT_MODE_POSITION = 0,
    RATETABLE_MOVEMENT_MODE_RATE = 1
} RATETABLE_MOVEMENT_MODE_ENUM;

typedef enum _RATETABLE_REALTIME_TYPE_INPUT_ENUM {
    RATETABLE_REALTIME_TYPE_INPUT_RS232 = 0,
    RATETABLE_REALTIME_TYPE_INPUT_RS422 = 1
} RATETABLE_REALTIME_TYPE_INPUT_ENUM;

struct ratetable_motion_data_t {
    int32_t hwil_input[3];
    int32_t hwil_output[3];
} __attribute__((packed));

struct ratetable_eqmt_info_t {
    uint32_t hwil_ratio_data;
    uint8_t hwil_type_comm_input;
    uint32_t hwil_tx_speed_baud;
    uint32_t hwil_send_freq;
    uint8_t movement_mode;
    float sample_time;
    struct ratetable_motion_data_t mot_data;
};

#ifdef __cplusplus
extern "C" {
#endif

int ratetable_init(struct ratetable_eqmt_info_t *eqmt);
int ratetable_init_input_file(FILE **stream);
int ratetable_convert_csv_to_motdata(struct ratetable_eqmt_info_t *eqmt, FILE *stream);
int ratetable_layer2_frame_direct_transfer(struct icf_ctrlblk_t *C, struct ratetable_eqmt_info_t *eqmt);
int ratetable_layer2_frame_received(struct icf_ctrlblk_t *C, struct ratetable_eqmt_info_t *eqmt);
#ifdef __cplusplus
}
#endif

#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_RATETABLE_H_
