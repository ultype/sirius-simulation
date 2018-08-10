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

    {FC_to_PFS_III, PFS_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_FILLING, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_FLY_MODE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_24V_PWR, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_BALL_VALVES_ONOFF_FAKE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_III, PFS_BALL_VALVES_ONOFF_REAL, EGSE_RX_FLIGHT_EVENT_QIDX},

    {FC_to_PFS_II, PFS_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_FILLING, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_FLY_MODE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_24V_PWR, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_FAKE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_REAL, EGSE_RX_FLIGHT_EVENT_QIDX},

    {FC_to_RCS_III, RCS_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_RCS_III, RCS_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_RCS_III, RCS_START, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_RCS_III, RCS_STOP, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_RCS_III, RCS_BALL_VALVES_ONOFF_FAKE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_RCS_III, RCS_BALL_VALVES_ONOFF_REAL, EGSE_RX_FLIGHT_EVENT_QIDX},

    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_READY_OPEN_FAIRING, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_DISABLE_FAIRING_FUNC, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_FAKE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_REAL, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_READY_DEPLOY_PAYLOAD, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DISABLE_DEPLOY_FUNC, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_PAYLOAD, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO1, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO2, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO3, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_PAYLOAD_CTRL_DEPLOY_CUBESAT_NO4, EGSE_RX_FLIGHT_EVENT_QIDX},

    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_REPORT_STATUS, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_BUILD_IN_TEST, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_CTRL_READY_TO_SEPARATE, EGSE_RX_FLIGHT_EVENT_QIDX},
    {FC_to_ORDNANCE_SEPARATION_II, ORDNANCE_SEPARATION_CTRL_SEPARATE_II_and_III, EGSE_RX_FLIGHT_EVENT_QIDX}
};

static struct fc_can_hash_table *p_fc_can_hash_object;

struct fc_can_hash_table *fc_can_hash_table_create(void) {
    struct fc_can_hash_table *object;
    int idx = 0;
    object = malloc(sizeof(struct fc_can_hash_table));
    if (object == NULL)
        goto fail;
    for (idx = 0; idx < FC_CAN_HASHTBL_SIZE; idx++) {
        object->bucket[idx] = malloc(sizeof(struct fc_can_hash_entry));
        memset(object->bucket[idx], 0 , sizeof(struct fc_can_hash_entry));
    }
    if (object->bucket == NULL) {
            fprintf(stderr, "[%s:%d] Memory allocate fail !!\n", __FUNCTION__, __LINE__);
            goto fail;
    }
    object->size = FC_CAN_HASHTBL_SIZE;
    return object;
fail:
    if (object->bucket)
        free(object->bucket[idx]);
    if (object)
        free(object);
    return NULL;
}

void fc_can_hash_table_dump(struct fc_can_hash_table *object) {
    int idx = 0;
    struct fc_can_hash_entry *curr;
    for (idx = 0; idx < FC_CAN_HASHTBL_SIZE; idx++) {
        curr = object->bucket[idx];
        printf("[%d]:", idx);
        while (curr != NULL) {
            printf("0x%x, ", curr->key);
            curr = curr->next;
        }
        printf("\n");
    }
}

struct fc_can_info_t *fc_can_hash_entry_find(struct fc_can_hash_table *object, uint32_t canid, uint8_t taskcmd) {
    struct fc_can_hash_entry *curr;
    uint8_t hash_idx;
    uint64_t find_key;
    hash_idx = FLIGHT_EVENT_HASH_INDEX(canid, taskcmd);
    find_key = (((uint64_t)canid << 8) & 0xFFFFFFFF00) | ((uint64_t)taskcmd & 0xFF);
    curr = object->bucket[hash_idx];
    while (curr->next != NULL) {
        if (curr->key == find_key) {
            return curr->data;
        }
        curr = curr->next;
    }
    return NULL;
}

int fc_can_hash_entry_add(struct fc_can_hash_table *object, struct fc_can_info_t *info) {
    struct fc_can_hash_entry *curr;
    uint8_t hash_idx;
    hash_idx = FLIGHT_EVENT_HASH_INDEX(info->canid, info->taskcmd);

    curr = object->bucket[hash_idx];
    while (curr->next != NULL) {
        curr = curr->next;
    }
    curr->next = malloc(sizeof(struct fc_can_hash_entry));
    if (curr->next == NULL) {
            fprintf(stderr, "[%s:%d] Memory allocate fail !!\n", __FUNCTION__, __LINE__);
            return -1;
    }
    memset(curr->next, 0 , sizeof(struct fc_can_hash_entry));
    curr->key = (((uint64_t)info->canid << 8) & 0xFFFFFFFF00) | ((uint64_t)info->taskcmd & 0xFF);
    curr->data = info;
    curr->next->next = NULL;
    return hash_idx;
}

int fc_can_hashtbl_init(void) {
    p_fc_can_hash_object = fc_can_hash_table_create();
    int idx = 0;
    uint8_t hash_idx;
    int cmd_count = sizeof(cmd_dispatch_map) /sizeof(struct fc_can_info_t);
    for (idx = 0; idx < cmd_count; idx++) {
        hash_idx = fc_can_hash_entry_add(p_fc_can_hash_object, &cmd_dispatch_map[idx]);
    }
    fc_can_hash_table_dump(p_fc_can_hash_object);
    return 0;
}


int fc_can_cmd_dispatch(void *rxframe) {
    int qidx = EGSE_EMPTY_SW_QIDX;
    struct can_frame *pframe = NULL;
    struct fc_can_info_t *info = NULL;
    pframe = (struct can_frame *)rxframe;
    info = fc_can_hash_entry_find(p_fc_can_hash_object, pframe->can_id, pframe->data[0]);
    if (info == NULL)
        fprintf(stderr, "[%s:%d] HASH COMMAND Not Found in Table\n", __FUNCTION__, __LINE__);
    else
        qidx = info->sw_queue_idx;
    return qidx;
}
