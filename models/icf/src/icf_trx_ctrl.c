#include "icf_trx_ctrl.h"

static const struct icf_mapping g_icf_egse_maptbl[] = {
    {TVC_SW_QIDX,                HW_PORT0, ICF_DRIVERS_ID0},
    {IMU01_SW_QIDX,              HW_PORT1, ICF_DRIVERS_ID1},
    {RATETBL_X_SW_QIDX,          HW_PORT2, ICF_DRIVERS_ID1},
    {RATETBL_Y_SW_QIDX,          HW_PORT3, ICF_DRIVERS_ID1},
    {RATETBL_Z_SW_QIDX,          HW_PORT4, ICF_DRIVERS_ID1},
    {IMU02_SW_QIDX,              HW_PORT5, ICF_DRIVERS_ID1},
    {GPSR01_SW_QIDX,             HW_PORT6, ICF_DRIVERS_ID1},
    {GPSR02_SW_QIDX,             HW_PORT7, ICF_DRIVERS_ID1},
    {FLIGHT_COMPUTER_SW_QIDX,    HW_PORT8, ICF_DRIVERS_ID2}
};


static struct icf_ctrl_port g_egse_port[] = {
    {1, HW_PORT0, "can0",        EMPTY_NETPORT,   CAN_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT1, "/dev/ttyAP0", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT2, "/dev/ttyAP1", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT3, "/dev/ttyAP2", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT4, "/dev/ttyAP3", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {0, HW_PORT5, "/dev/ttyAP4", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {0, HW_PORT6, "/dev/ttyAP5", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT7, "/dev/ttyAP6", EMPTY_NETPORT, RS422_DEVICE_TYPE,     NULL, NULL},
    {1, HW_PORT8, "127.0.0.1",   8700,          ETHERNET_DEVICE_TYPE,  NULL, NULL}
};

static struct icf_ctrl_queue g_egse_queue[] = {
    {TVC_SW_QIDX,              ICF_DIRECTION_RX, &g_egse_port[HW_PORT0]},
    {IMU01_SW_QIDX,            ICF_DIRECTION_TX, &g_egse_port[HW_PORT1]},
    {RATETBL_X_SW_QIDX,        ICF_DIRECTION_TX, &g_egse_port[HW_PORT2]},
    {RATETBL_Y_SW_QIDX,        ICF_DIRECTION_TX, &g_egse_port[HW_PORT3]},
    {RATETBL_Z_SW_QIDX,        ICF_DIRECTION_TX, &g_egse_port[HW_PORT4]},
    {IMU02_SW_QIDX,            ICF_DIRECTION_TX, &g_egse_port[HW_PORT5]},
    {GPSR01_SW_QIDX,           ICF_DIRECTION_TX, &g_egse_port[HW_PORT6]},
    {GPSR02_SW_QIDX,           ICF_DIRECTION_TX, &g_egse_port[HW_PORT7]},
    {FLIGHT_COMPUTER_SW_QIDX,  ICF_DIRECTION_TX, &g_egse_port[HW_PORT8]}
};


static const struct icf_mapping g_icf_esps_maptbl[] = {
    {TVC_SW_QIDX,                HW_PORT0, ICF_DRIVERS_ID0}
};


static struct icf_ctrl_port g_esps_port[] = {
    {1, HW_PORT0, "can1",        EMPTY_NETPORT,   CAN_DEVICE_TYPE,     NULL, NULL}
};

static struct icf_ctrl_queue g_esps_queue[] = {
    {TVC_SW_QIDX,              ICF_DIRECTION_TX, &g_esps_port[HW_PORT0]}
};



static int icf_qidx_to_drivers_id(int qidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = &g_icf_egse_maptbl;
    int    map_tbl_size = sizeof(g_icf_egse_maptbl);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        which_map_tbl = g_icf_esps_maptbl;
        map_tbl_size = sizeof(g_icf_esps_maptbl);
    }
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].driver_id;
}

static int icf_pidx_to_drivers_id(int pidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = &g_icf_egse_maptbl;
    int    map_tbl_size = sizeof(g_icf_egse_maptbl);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        which_map_tbl = g_icf_esps_maptbl;
        map_tbl_size = sizeof(g_icf_esps_maptbl);
    }
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].hw_port == pidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].driver_id;
}

static int icf_pidx_to_qidx(int pidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = &g_icf_egse_maptbl;
    int    map_tbl_size = sizeof(g_icf_egse_maptbl);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        which_map_tbl = g_icf_esps_maptbl;
        map_tbl_size = sizeof(g_icf_esps_maptbl);
    }
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].hw_port == pidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].sw_queue;
}

static int icf_qidx_to_pidx(int qidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = &g_icf_egse_maptbl;
    int    map_tbl_size = sizeof(g_icf_egse_maptbl);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        which_map_tbl = g_icf_esps_maptbl;
        map_tbl_size = sizeof(g_icf_esps_maptbl);
    }
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].hw_port;
}

int icf_ctrlblk_init(struct icf_ctrlblk_t* C, int system_type) {
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport;
    struct icf_driver_ops *drv_ops;
    int idx;
    struct icf_ctrl_queue *queue_blk_tbl_p = &g_egse_queue;
    int queue_blk_size = sizeof(g_egse_queue);
    struct icf_ctrl_port *port_blk_tbl_p = &g_egse_port;
    int port_blk_size = sizeof(g_egse_port);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        queue_blk_tbl_p = &g_esps_queue;
        queue_blk_size = sizeof(g_esps_queue);
        port_blk_tbl_p = &g_esps_port;
        port_blk_size = sizeof(g_esps_port);
    }

    for (idx = 0; idx < get_arr_num(queue_blk_size, sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &queue_blk_tbl_p[idx];
        rb_init(&ctrlqueue->data_ring, NUM_OF_CELL);
        C->ctrlqueue[idx] = ctrlqueue;
    }
    for (idx = 0; idx < get_arr_num(port_blk_size, sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &port_blk_tbl_p[idx];
        C->ctrlport[idx] = ctrlport;
        ctrlport->drv_priv_ops = icf_drivers[icf_pidx_to_drivers_id(idx, system_type)];
        drv_ops = ctrlport->drv_priv_ops;
        if (ctrlport->enable == 0)
            continue;
        drv_ops->open_interface(&ctrlport->drv_priv_data, ctrlport->ifname, ctrlport->netport);
    }
    return 0;
}

int icf_ctrlblk_deinit(struct icf_ctrlblk_t* C, int system_type) {
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport;
    struct icf_driver_ops *drv_ops;
    int idx;

    struct icf_ctrl_queue *queue_blk_tbl_p = &g_egse_queue;
    struct icf_ctrl_port *port_blk_tbl_p = &g_egse_port;

    int port_blk_size = sizeof(g_egse_port);
    int queue_blk_size = sizeof(g_egse_queue);

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        queue_blk_tbl_p = &g_esps_queue;
        queue_blk_size = sizeof(g_esps_queue);
        port_blk_tbl_p = &g_esps_port;
        port_blk_size = sizeof(g_esps_port);
    }
    for (idx = 0; idx < get_arr_num(queue_blk_size, sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &queue_blk_tbl_p[idx];
        rb_deinit(&ctrlqueue->data_ring);
        C->ctrlqueue[idx] = NULL;
    }
    for (idx = 0; idx < get_arr_num(port_blk_size, sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &port_blk_tbl_p[idx];
        drv_ops = ctrlport->drv_priv_ops;
        if (ctrlport->enable == 0)
            continue;
        drv_ops->close_interface(&ctrlport->drv_priv_data);
    }
    return 0;
}

int icf_rx_dequeue(struct icf_ctrlblk_t* C, int qidx, void **payload, uint32_t size) {
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    *payload = rb_pop(&ctrlqueue->data_ring);
    debug_hex_dump("icf_rx_dequeue", payload, size);
    return 0;
}

static int icf_dispatch_rx_frame(uint32_t frame_id) {
    int qidx = 0;
    switch (frame_id) {
        case FC2TVC_III_NO1:
        case FC2TVC_III_NO2:
        case FC2TVC_II_NO1:
        case FC2TVC_II_NO2:
            qidx = TVC_SW_QIDX;
            break;
        case FC2VALVE_III_NO1:
        case FC2RCS_III:
        case FC2ORDNANCE_FAIRING_III:
        case FC2VALVE_II_NO1:
        case FC2ORDNANCE_SEPARATION_II:
            qidx = EMPTY_SW_QIDX;
            break;
        default:
            fprintf(stderr, "[%s] Unknown CAN command. ID = 0x%x\n", __FUNCTION__, frame_id);
            qidx = EMPTY_SW_QIDX;
    }
    return qidx;
}

int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int pidx) {
    struct timeval tv;
    int ret;
    int qidx = 0;
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport = C->ctrlport[pidx];
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    struct can_frame *pframe = NULL;
    tv.tv_sec = 0;
    tv.tv_usec = 100;

    drv_ops->fd_zero(ctrlport->drv_priv_data);
    drv_ops->fd_set(ctrlport->drv_priv_data);
    ret = drv_ops->select(ctrlport->drv_priv_data, &tv);
    if (ret < 0)
        fprintf(stderr, "select error: %d\n", ret);
    /*TODO malloc need use the static memory*/
    if (drv_ops->fd_isset(ctrlport->drv_priv_data)) {
        pframe = NULL;
        pframe = icf_alloc_mem(sizeof(struct can_frame));
        if (pframe == NULL) {
            fprintf(stderr, "producer frame allocate fail!!\n");
        }
        if (drv_ops->recv_data(ctrlport->drv_priv_data, (uint8_t *)pframe, sizeof(struct can_frame)) > 0) {
            qidx = icf_dispatch_rx_frame(pframe->can_id);
            if (qidx > EMPTY_SW_QIDX) {
                ctrlqueue = C->ctrlqueue[qidx];
                FTRACE_TIME_STAMP(ctrlqueue->queue_idx + 500);
                rb_push(&ctrlqueue->data_ring, pframe);
            }
        }
        debug_print("[%lf] RX CAN Received !!\n", get_curr_time());
        debug_hex_dump("icf_rx_ctrl_job", (uint8_t *)pframe, sizeof(struct can_frame));
    }
    return 0;
}


int icf_tx_direct(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;

    frame_full_size = size;
    if (drv_ops->get_header_size) {
        frame_full_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
    }

    tx_buffer = icf_alloc_mem(frame_full_size);

    if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
        drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload, size);
        offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
    }
    memcpy(tx_buffer + offset, (uint8_t *) payload, size);
    drv_ops->send_data(ctrlport->drv_priv_data, tx_buffer, frame_full_size);
    debug_hex_dump("icf_tx_direct", tx_buffer, frame_full_size);
    icf_free_mem(tx_buffer);
    return 0;
}

int icf_tx_enqueue(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct ringbuffer_cell_t *txcell;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;

    whichring = &ctrlqueue->data_ring;
    frame_full_size = size;
    if (drv_ops->get_header_size) {
        frame_full_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
    }
    tx_buffer = icf_alloc_mem(frame_full_size);

    if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
        drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload, size);
        offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
    }
    memcpy(tx_buffer + offset, (uint8_t *) payload, size);

    if (whichring) {
        txcell = icf_alloc_mem(sizeof(struct ringbuffer_cell_t));
        txcell->frame_full_size = frame_full_size;
        txcell->l2frame = tx_buffer;
        rb_push(whichring, txcell);
    }

    return 0;
}

int icf_tx_ctrl_job(struct icf_ctrlblk_t* C, int qidx) {
    struct ringbuffer_cell_t *txcell = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    whichring = &ctrlqueue->data_ring;
    txcell = (uint8_t *)rb_pop(whichring);
    FTRACE_TIME_STAMP(ctrlqueue->queue_idx + 500);
    if (txcell) {
        drv_ops->send_data(ctrlport->drv_priv_data, txcell->l2frame, txcell->frame_full_size);
        debug_hex_dump("icf_tx_ctrl_job", txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
    }

    return 0;
}

void icf_heartbeat(void) {
    fprintf(stderr, "*");
}

void *icf_alloc_mem(size_t size) {
    return calloc(1, size);
}


void icf_free_mem(void *ptr) {
    if (ptr) {
        free(ptr);
        ptr = NULL;
    }
    return;
}
