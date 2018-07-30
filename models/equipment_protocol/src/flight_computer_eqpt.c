#include "flight_computer_eqpt.h"

int fc_can_cmd_tvc_box(struct can_frame *pframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    switch(pframe->data[0]) {
        case TVC_REPORT_STATUS:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case TVC_BUILD_IN_TEST:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case TVC_START:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case TVC_STOP:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case TVC_MOVEMENT_FAKE:
        case TVC_MOVEMENT_REAL:
            qidx = EGSE_TVC_SW_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown TASK command. TVC ID = 0x%x TASK = 0x%x\n", __FUNCTION__, pframe->can_id, pframe->data[0]);
            qidx = EGSE_EMPTY_SW_QIDX;

    }
    return qidx;
}

int fc_can_cmd_pfs_box(struct can_frame *pframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    switch(pframe->data[0]) {
        case PFS_REPORT_STATUS:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case PFS_BUILD_IN_TEST:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case PFS_START:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case PFS_STOP:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case PFS_BALL_VALVES_ONOFF_FAKE:
        case PFS_BALL_VALVES_ONOFF_REAL:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown TASK command. PFS ID = 0x%x TASK = 0x%x\n", __FUNCTION__, pframe->can_id, pframe->data[0]);
            qidx = EGSE_EMPTY_SW_QIDX;

    }
    return qidx;
}

int fc_can_cmd_rcs_box(struct can_frame *pframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    switch(pframe->data[0]) {
        case RCS_REPORT_STATUS:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case RCS_BUILD_IN_TEST:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case RCS_START:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case RCS_STOP:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case RCS_BALL_VALVES_ONOFF_FAKE:
        case RCS_BALL_VALVES_ONOFF_REAL:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown TASK command. RCS ID = 0x%x TASK = 0x%x\n", __FUNCTION__, pframe->can_id, pframe->data[0]);
            qidx = EGSE_EMPTY_SW_QIDX;

    }
    return qidx;
}

int fc_can_cmd_ordnance_fairing_box(struct can_frame *pframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    switch(pframe->data[0]) {
        case ORDNANCE_FAIRING_REPORT_STATUS:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_BUILD_IN_TEST:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_NOSECONE_CTRL_READY_OPEN_FAIRING:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_NOSECONE_CTRL_DISABLE_FAIRING_FUNC:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_FAKE:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_REAL:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_READY_DEPLOY_PAYLOAD:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DISABLE_DEPLOY_FUNC:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_PAYLOAD:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO1:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO2:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO3:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO4:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
        default:
            fprintf(stderr, "[%s] Unknown TASK command. ordnance_fairing ID = 0x%x TASK = 0x%x\n", __FUNCTION__, pframe->can_id, pframe->data[0]);
            qidx = EGSE_EMPTY_SW_QIDX;

    }
    return qidx;
}

int fc_can_cmd_ordnance_separation_box(struct can_frame *pframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    switch(pframe->data[0]) {
        case ORDNANCE_SEPARATION_REPORT_STATUS:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_SEPARATION_BUILD_IN_TEST:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_SEPARATION_CTRL_READY_TO_SEPARATE:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        case ORDNANCE_SEPARATION_CTRL_SEPARATE_II_and_III:
            qidx = EGSE_RX_FLIGHT_EVENT_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown TASK command. ordnance_separation ID = 0x%x TASK = 0x%x\n", __FUNCTION__, pframe->can_id, pframe->data[0]);
            qidx = EGSE_EMPTY_SW_QIDX;

    }
    return qidx;
}


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
