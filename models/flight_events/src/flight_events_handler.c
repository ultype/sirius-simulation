#include "flight_events_handler.h"
#include "flight_events_define.h"

struct flight_event_can_info_t flight_event_can_mapping_tbl[] = {
    {0, FC_to_TVC_II_NO1, TVC_START, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_LIFTOFF},
    {0, FC_to_PFS_III, PFS_24V_PWR,  {0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_HOT_STAGING},
    {0, FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_REAL, {0xBC, 0x23, 0xAD, 0x14, 0xAA, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_S3_SEPERATION},
    {0, FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_REAL, {0xDC, 0x43, 0xBA, 0x21, 0xAA, 0xFF, 0xFF}, FLIGHT_EVENT_FAIRING_JETTSION}
};

uint64_t flight_event_code_search(struct can_frame* receive) {
    struct flight_event_can_info_t *info;
    int tbl_size = sizeof(flight_event_can_mapping_tbl) / sizeof(struct flight_event_can_info_t);
    int idx = 0;
    uint32_t bitmap = 0x0;

    while (idx < tbl_size) {
        info = &flight_event_can_mapping_tbl[idx];
        bitmap = 0x0;
        bitmap |= ((info->canid == receive->can_id ? 0x1 : 0) << 0);
        bitmap |= ((info->taskcmd == receive->data[0] ? 0x1 : 0) << 1);
        bitmap |= ((memcmp(&info->content[0], &receive->data[1], 7) == 0 ? 0x1 : 0) << 2);
        if (bitmap == 0x7) {
            return info->event_code;
        }
        idx++;
    }
    return 0;
}

int flight_event_code_handler(struct icf_ctrlblk_t *C, uint64_t *event_code, int system_type) {
    struct can_frame receive_event;
    switch (system_type) {
        case ICF_SYSTEM_TYPE_SIL_EGSE:
            break;
        case ICF_SYSTEM_TYPE_EGSE:
                if (icf_rx_dequeue(C, EGSE_RX_FLIGHT_EVENT_QIDX, &receive_event, sizeof(struct can_frame)) > 0) {
                    *event_code = flight_event_code_search(&receive_event);
                }
            break;
        default:
            fprintf(stderr, "Unknown System type %d !!!\n", system_type);
    }
    return 0;
}
