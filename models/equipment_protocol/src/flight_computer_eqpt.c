#include "flight_computer_eqpt.h"

static const  struct fc_can_info_t cmd_dispatch_map[] = {
    {FC_to_TVC_III_NO1, TVC_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO1, TVC_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO1, TVC_START, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO1, TVC_STOP, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO1, TVC_MOVEMENT_FAKE, EGSE_TVC_SW_QIDX},
    {FC_to_TVC_III_NO1, TVC_MOVEMENT_REAL, EGSE_TVC_SW_QIDX},

    {FC_to_TVC_III_NO2, TVC_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO2, TVC_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO2, TVC_START, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO2, TVC_STOP, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_III_NO2, TVC_MOVEMENT_FAKE, EGSE_TVC_SW_QIDX},
    {FC_to_TVC_III_NO2, TVC_MOVEMENT_REAL, EGSE_TVC_SW_QIDX},

    {FC_to_TVC_II_NO1, TVC_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO1, TVC_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO1, TVC_START, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO1, TVC_STOP, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO1, TVC_MOVEMENT_FAKE, EGSE_TVC_SW_QIDX},
    {FC_to_TVC_II_NO1, TVC_MOVEMENT_REAL, EGSE_TVC_SW_QIDX},

    {FC_to_TVC_II_NO2, TVC_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO2, TVC_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO2, TVC_START, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO2, TVC_STOP, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_TVC_II_NO2, TVC_MOVEMENT_FAKE, EGSE_TVC_SW_QIDX},
    {FC_to_TVC_II_NO2, TVC_MOVEMENT_REAL, EGSE_TVC_SW_QIDX},

    {FC_to_PFS_III, PFS_REPORT_STATUS, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_III, PFS_BUILD_IN_TEST, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_III, PFS_START, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_III, PFS_STOP, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_III, PFS_BALL_VALVES_ONOFF_FAKE, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_III, PFS_BALL_VALVES_ONOFF_REAL, EGSE_EMPTY_SW_QIDX},

    {FC_to_PFS_II, PFS_REPORT_STATUS, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_II, PFS_BUILD_IN_TEST, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_II, PFS_START, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_II, PFS_STOP, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_FAKE, EGSE_EMPTY_SW_QIDX},
    {FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_REAL, EGSE_EMPTY_SW_QIDX},

    {FC_to_RCS_III, RCS_REPORT_STATUS, EGSE_EMPTY_SW_QIDX},
    {FC_to_RCS_III, RCS_BUILD_IN_TEST, EGSE_EMPTY_SW_QIDX},
    {FC_to_RCS_III, RCS_START, EGSE_EMPTY_SW_QIDX},
    {FC_to_RCS_III, RCS_STOP, EGSE_EMPTY_SW_QIDX},
    {FC_to_RCS_III, RCS_BALL_VALVES_ONOFF_FAKE, EGSE_EMPTY_SW_QIDX},
    {FC_to_RCS_III, RCS_BALL_VALVES_ONOFF_REAL, EGSE_EMPTY_SW_QIDX},

    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_REPORT_STATUS, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_BUILD_IN_TEST, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_READY_OPEN_FAIRING, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_DISABLE_FAIRING_FUNC, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_FAKE, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_REAL, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_READY_DEPLOY_PAYLOAD, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DISABLE_DEPLOY_FUNC, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_PAYLOAD, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO1, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO2, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO3, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO4, EGSE_EMPTY_SW_QIDX},

    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_REPORT_STATUS, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_BUILD_IN_TEST, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_CTRL_READY_TO_SEPARATE, EGSE_EMPTY_SW_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_CTRL_SEPARATE_II_and_III, EGSE_EMPTY_SW_QIDX}
};

static struct fc_can_hash_table *g_fc_can_hash_object;

struct fc_can_hash_table *fc_can_hash_table_create(void) {
    struct fc_can_hash_table *object;
    object = malloc(sizeof(struct fc_can_hash_table));
    if (object == NULL)
        goto fail;
    object->bucket = malloc(sizeof(struct fc_can_hash_entry*) * FC_CAN_HASHTBL_SIZE);
    if (object == NULL)
        goto fail;
    memset(object->bucket, 0 , sizeof(struct hash_fc_can_hash_entryentry*) * FC_CAN_HASHTBL_SIZE);
    object->size = FC_CAN_HASHTBL_SIZE;
    return object;
fail:
    if (object->bucket)
        free(object->bucket);
    if (object)
        free(object);
    return NULL;
}

int fc_can_hash_entry_find(struct fc_can_hash_table *object, uint32_t canid, uint8_t taskcmd) {
    struct fc_can_hash_entry **curr;
    uint8_t hash_idx;
    hash_idx = FLIGHT_EVENT_HASH_INDEX(canid, taskcmd);
    return hash_idx;
}

int fc_can_hash_entry_add(struct fc_can_hash_table *object, struct fc_can_info_t *info) {
    struct fc_can_hash_entry **curr;
    uint8_t hash_idx;
    hash_idx = FLIGHT_EVENT_HASH_INDEX(info->canid, info->taskcmd);
    return hash_idx;
}

int fc_can_hashtbl_init(void) {
    g_fc_can_hash_object = fc_can_hash_table_create();
    int idx = 0;
    uint8_t hash_idx;
    int cmd_count = sizeof(cmd_dispatch_map) /sizeof(struct fc_can_info_t);
    for (idx = 0; idx < cmd_count; ++idx) {
        hash_idx = fc_can_hash_entry_add(g_fc_can_hash_object, &cmd_dispatch_map[idx]);
        printf("[%d] hash_idx: %d\n", idx, hash_idx);
    }
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
