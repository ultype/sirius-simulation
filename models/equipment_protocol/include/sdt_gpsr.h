#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_GPSR_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_GPSR_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      SDT GPSR
LIBRARY DEPENDENCY:
      (
        (../src/sdt_gpsr.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stdint.h>
#include "icf_trx_ctrl.h"
#include "gpsr_s_nav_tlm.h"
struct sdt_gpsr_motion_data_t {
    uint16_t gps_week_num;
    uint32_t gps_time;
    uint16_t validity;
    float posx;
    float posy;
    float posz;
    float velx;
    float vely;
    float velz;
    uint32_t visibility_satellites_map;
};

#ifdef __cplusplus
extern "C" {
#endif

int sdt_gpsr_layer2_tlm_frame_transfer(struct icf_ctrlblk_t *C);
int sdt_gpsr_convert_motion_data_to_tlm(void *data , struct gpsr_s_nav_tlm_frame_t *tlm_frame);

#ifdef __cplusplus
}
#endif

#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SDT_GPSR_H_