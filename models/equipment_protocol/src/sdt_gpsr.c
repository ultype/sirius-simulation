#include "sdt_gpsr.h"
#include "gpsr_s_nav_tlm.h"
#include <string.h>

int sdt_gpsr_convert_motion_data_to_tlm(void *data , struct gpsr_s_nav_tlm_frame_t *tlm_frame) {
    struct sdt_gpsr_motion_data_t *mot_data = (struct sdt_gpsr_motion_data_t *)data;

    memset(tlm_frame,0 , sizeof(struct gpsr_s_nav_tlm_frame_t));
    tlm_frame->gps_week_num = mot_data->gps_week_num;
    tlm_frame->gps_time = mot_data->gps_time;
    tlm_frame->validity = mot_data->validity;
    tlm_frame->posx = mot_data->posx;
    tlm_frame->posy = mot_data->posy;
    tlm_frame->posz = mot_data->posz;
    tlm_frame->velx = mot_data->velx;
    tlm_frame->vely = mot_data->vely;
    tlm_frame->velz = mot_data->velz;
    tlm_frame->visibility_satellites_map = mot_data->visibility_satellites_map;
    return 0;
}

int sdt_gpsr_layer2_tlm_frame_transfer(struct icf_ctrlblk_t *C) {
    struct gpsr_s_nav_tlm_frame_t tlm_frame;
    uint32_t send_size = sizeof(struct gpsr_s_nav_tlm_frame_t);
    icf_tx_dequeue(C, EGSE_GPSR01_SW_QIDX, &tlm_frame);
    icf_tx_direct(C, EGSE_GPSR01_SW_QIDX, &tlm_frame, send_size);
    //  icf_tx_direct(C, EGSE_GPSR02_SW_QIDX, &tlm_frame, send_size);
    //  hex_dump("TX TLM Frame", (uint8_t *) &tlm_frame, sizeof(struct gpsr_s_nav_tlm_frame_t));
    return 0;
}
