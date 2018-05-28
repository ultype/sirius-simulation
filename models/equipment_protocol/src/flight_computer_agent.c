#include "flight_computer_agent.h"

int fc_can_cmd_dispatch(void *rxframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    struct can_frame *pframe = NULL;
    pframe = (struct can_frame *)rxframe;
    switch (pframe->can_id) {
        case FC2TVC_III_NO1:
        case FC2TVC_III_NO2:
        case FC2TVC_II_NO1:
        case FC2TVC_II_NO2:
            qidx = EGSE_TVC_SW_QIDX;
            break;
        case FC2VALVE_III_NO1:
        case FC2RCS_III:
        case FC2ORDNANCE_FAIRING_III:
        case FC2VALVE_II_NO1:
        case FC2ORDNANCE_SEPARATION_II:
            qidx = EGSE_EMPTY_SW_QIDX;
            break;
        case 0x555:
            qidx = EGSE_RX_MISSION_EVENT_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown CAN command. ID = 0x%x\n", __FUNCTION__, pframe->can_id);
            qidx = EGSE_EMPTY_SW_QIDX;
    }
    return qidx;
}
