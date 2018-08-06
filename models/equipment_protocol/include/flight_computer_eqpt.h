/********************************* TRICK HEADER *******************************
PURPOSE:
      Flight Computer Agent
LIBRARY DEPENDENCY:
      (
        (../src/flight_computer_eqpt.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_FLIGHT_COMPUTER_EQPT_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_FLIGHT_COMPUTER_EQPT_H_
#include "icf_trx_ctrl.h"
#include "dsp_can_interfaces.h"

#define FC_CAN_HASHTBL_SIZE 256
#define GET_BYTE(value, shift_bit) (((value >> shift_bit) & 0xFF))
#define CANID_TASKCMD_HASH(canid, taskcmd)  (GET_BYTE(canid, 24) ^ GET_BYTE(canid, 16) ^ GET_BYTE(canid, 8) ^ GET_BYTE(canid, 0) ^ (uint8_t)taskcmd)
#define FLIGHT_EVENT_HASH_INDEX(canid, taskcmd) (CANID_TASKCMD_HASH(canid, taskcmd)  & (FC_CAN_HASHTBL_SIZE - 1))



struct fc_can_info_t {
    uint32_t canid;
    uint8_t taskcmd;
    int sw_queue_idx;
};

struct fc_can_hash_entry {
    uint64_t key;
    struct fc_can_info_t *data;
    struct fc_can_hash_entry *next;
};

struct fc_can_hash_table {
    int size;
    struct fc_can_hash_entry *bucket[FC_CAN_HASHTBL_SIZE];
};

#ifdef __cplusplus
extern "C" {
#endif
int fc_can_cmd_dispatch(void *rxframe);
int fc_can_hashtbl_init(void);
#ifdef __cplusplus
}
#endif
#endif   //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_FLIGHT_COMPUTER_EQPT_H_
