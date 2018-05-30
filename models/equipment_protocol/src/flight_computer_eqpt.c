#include "flight_computer_eqpt.h"

int fc_can_cmd_dispatch(void *rxframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    struct can_frame *pframe = NULL;
    pframe = (struct can_frame *)rxframe;
    switch (pframe->can_id) {
        case FC_to_TVC_III_NO1:
        case FC_to_TVC_III_NO2:
        case FC_to_TVC_II_NO1:
        case FC_to_TVC_II_NO2:
            if (pframe->data[0] == TVC_MOVEMENT_FAKE || pframe->data[0] == TVC_MOVEMENT_REAL) {
                qidx = EGSE_TVC_SW_QIDX;
            }
            break;
        case FC_to_PFS_III:
        case FC_to_RCS_III:
        case FC_to_ORDNANCE_FAIRING_III:
        case FC_to_PFS_II:
        case FC_to_ORDNANCE_SEPARATION_II:
            qidx = EGSE_EMPTY_SW_QIDX;
            break;
        case FC_to_EGSE_MISSION_EVENT_FAKE:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown CAN command. ID = 0x%x\n", __FUNCTION__, pframe->can_id);
            qidx = EGSE_EMPTY_SW_QIDX;
    }
    return qidx;
}
