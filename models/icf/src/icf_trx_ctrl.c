#include "icf_trx_ctrl.h"
#include "simgen_remote.h"
static const struct icf_mapping g_icf_egse_maptbl[] = {
    {HW_PORT0, EGSE_TVC_SW_QIDX,              ICF_DRIVERS_ID0},
    {HW_PORT1, EGSE_IMU01_SW_QIDX,            ICF_DRIVERS_ID1},
    {HW_PORT2, EGSE_RATETBL_X_SW_QIDX,        ICF_DRIVERS_ID1},
    {HW_PORT3, EGSE_RATETBL_Y_SW_QIDX,        ICF_DRIVERS_ID1},
    {HW_PORT4, EGSE_RATETBL_Z_SW_QIDX,        ICF_DRIVERS_ID1},
    {HW_PORT5, EGSE_IMU02_SW_QIDX,            ICF_DRIVERS_ID1},
    {HW_PORT6, EGSE_GPSR01_SW_QIDX,           ICF_DRIVERS_ID1},
    {HW_PORT7, EGSE_GPSR02_SW_QIDX,           ICF_DRIVERS_ID1},
    {HW_PORT8, EGSE_FLIGHT_COMPUTER_SW_QIDX,  ICF_DRIVERS_ID2},
    {HW_PORT0, EGSE_RX_FLIGHT_EVENT_QIDX,     ICF_DRIVERS_ID0},
    {HW_PORT9, EGSE_TX_GPSRF_EMU_QIDX,        ICF_DRIVERS_ID3},
    {HW_PORT2, EGSE_RX_RATETBL_X_SW_QIDX,     ICF_DRIVERS_ID1},
    {HW_PORT3, EGSE_RX_RATETBL_Y_SW_QIDX,     ICF_DRIVERS_ID1},
    {HW_PORT4, EGSE_RX_RATETBL_Z_SW_QIDX,     ICF_DRIVERS_ID1}
};


static struct icf_ctrl_port g_egse_port[] = {
    {CAN_PORT_EN,           HW_PORT0, "can0",        EMPTY_NETPORT,     CAN_DEVICE_TYPE,       NULL, NULL},
    {RS422_IMU_PORT_EN,     HW_PORT1, "/dev/ttyAP0", RS422_HEADER_CRC,  RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_RATETBL_PORT_EN, HW_PORT2, "/dev/ttyAP1", RS422_HEADER_NONE, RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_RATETBL_PORT_EN, HW_PORT3, "/dev/ttyAP2", RS422_HEADER_NONE, RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_RATETBL_PORT_EN, HW_PORT4, "/dev/ttyAP3", RS422_HEADER_NONE, RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_IMU_PORT_EN,     HW_PORT5, "/dev/ttyAP4", RS422_HEADER_CRC,  RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_GPSR_PORT_EN,    HW_PORT6, "/dev/ttyAP5", RS422_HEADER_NONE, RS422_DEVICE_TYPE,     NULL, NULL},
    {RS422_GPSR_PORT_EN,    HW_PORT7, "/dev/ttyAP6", RS422_HEADER_NONE, RS422_DEVICE_TYPE,     NULL, NULL},
    {ETH_FC_PORT_EN,        HW_PORT8, "egse_server", 8700,              ETHERNET_DEVICE_TYPE,  NULL, NULL},
    {ETH_SIMGEN_PORT_EN,    HW_PORT9, SIMGEN_IP,     SIMGEN_PORT,       ETHERNET_DEVICE_TYPE,  NULL, NULL}
};

static struct icf_ctrl_queue g_egse_queue[] = {
    {1, EGSE_TVC_SW_QIDX,              ICF_DIRECTION_RX, NULL, {}},
    {1, EGSE_IMU01_SW_QIDX,            ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_RATETBL_X_SW_QIDX,        ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_RATETBL_Y_SW_QIDX,        ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_RATETBL_Z_SW_QIDX,        ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_IMU02_SW_QIDX,            ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_GPSR01_SW_QIDX,           ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_GPSR02_SW_QIDX,           ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_FLIGHT_COMPUTER_SW_QIDX,  ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_RX_FLIGHT_EVENT_QIDX,     ICF_DIRECTION_RX, NULL, {}},
    {1, EGSE_TX_GPSRF_EMU_QIDX,        ICF_DIRECTION_TX, NULL, {}},
    {1, EGSE_RX_RATETBL_X_SW_QIDX,     ICF_DIRECTION_RX, NULL, {}},
    {1, EGSE_RX_RATETBL_Y_SW_QIDX,     ICF_DIRECTION_RX, NULL, {}},
    {1, EGSE_RX_RATETBL_Z_SW_QIDX,     ICF_DIRECTION_RX, NULL, {}}
};

static const struct icf_mapping g_icf_esps_maptbl[] = {
    {HW_PORT0, ESPS_TVC_SW_QIDX,          ICF_DRIVERS_ID0},
    {HW_PORT8, ESPS_GNC_SW_QIDX,          ICF_DRIVERS_ID2},
    {HW_PORT0, ESPS_TX_MISSION_CODE_QIDX, ICF_DRIVERS_ID0}
};


static struct icf_ctrl_port g_esps_port[] = {
    {1, HW_PORT0, "can1",        EMPTY_NETPORT,         CAN_DEVICE_TYPE,      NULL, NULL},
    {1, HW_PORT8, ICF_EGSE_CONNECT_IP,   8700,          ETHERNET_DEVICE_TYPE, NULL, NULL}
};

static struct icf_ctrl_queue g_esps_queue[] = {
    {1, ESPS_TVC_SW_QIDX,                  ICF_DIRECTION_TX, NULL, {}},
    {1, ESPS_GNC_SW_QIDX,                  ICF_DIRECTION_RX, NULL, {}},
    {1, ESPS_TX_MISSION_CODE_QIDX,         ICF_DIRECTION_TX, NULL, {}}
};

/*  SIL EGSE Table  */
static const struct icf_mapping g_icf_egse_sil_maptbl[] = {
    {HW_PORT8, EGSE_SIL_DOWNLINK_SW_QIDX,     ICF_DRIVERS_ID2},
    {HW_PORT8, EGSE_FLIGHT_COMPUTER_SW_QIDX,  ICF_DRIVERS_ID2}
};

static struct icf_ctrl_port g_egse_sil_port[] = {
    {1, HW_PORT8, "egse_server", 8700,  ETHERNET_DEVICE_TYPE,  NULL, NULL}
};

static struct icf_ctrl_queue g_egse_sil_queue[] = {
    {1, EGSE_SIL_DOWNLINK_SW_QIDX,     ICF_DIRECTION_RX, NULL, {}},
    {1, EGSE_FLIGHT_COMPUTER_SW_QIDX,  ICF_DIRECTION_TX, NULL, {}}
};

/*  SIL ESPS Table  */
static const struct icf_mapping g_icf_esps_sil_maptbl[] = {
    {HW_PORT8, ESPS_TVC_SW_QIDX, ICF_DRIVERS_ID2},
    {HW_PORT8, ESPS_GNC_SW_QIDX, ICF_DRIVERS_ID2}
};

static struct icf_ctrl_port g_esps_sil_port[] = {
    {1, HW_PORT8, ICF_EGSE_CONNECT_IP,   8700,  ETHERNET_DEVICE_TYPE, NULL, NULL}
};

static struct icf_ctrl_queue g_esps_sil_queue[] = {
    {1, ESPS_TVC_SW_QIDX,                  ICF_DIRECTION_TX, NULL, {}},
    {1, ESPS_GNC_SW_QIDX,                  ICF_DIRECTION_RX, NULL, {}}
};

static struct icf_mapping *icf_choose_map_tbl(int system_type, int *tbl_size) {
    struct icf_mapping *table = NULL;

    switch (system_type) {
        case ICF_SYSTEM_TYPE_EGSE:
            table = g_icf_egse_maptbl;
            *tbl_size = sizeof(g_icf_egse_maptbl);
            break;
        case ICF_SYSTEM_TYPE_ESPS:
            table = g_icf_esps_maptbl;
            *tbl_size = sizeof(g_icf_esps_maptbl);
            break;
        case ICF_SYSTEM_TYPE_SIL_EGSE:
            table = g_icf_egse_sil_maptbl;
            *tbl_size = sizeof(g_icf_egse_sil_maptbl);
            break;
        case ICF_SYSTEM_TYPE_SIL_ESPS:
            table = g_icf_esps_sil_maptbl;
            *tbl_size = sizeof(g_icf_esps_sil_maptbl);
            break;
        default:
            table = g_icf_egse_maptbl;
            *tbl_size = sizeof(g_icf_egse_maptbl);
    }
    return table;
}

static struct icf_ctrl_queue *icf_choose_sw_queue_tbl(int system_type, int *tbl_size) {
    struct icf_ctrl_queue *table = NULL;

    switch (system_type) {
        case ICF_SYSTEM_TYPE_EGSE:
            table = g_egse_queue;
            *tbl_size = sizeof(g_egse_queue);
            break;
        case ICF_SYSTEM_TYPE_ESPS:
            table = g_esps_queue;
            *tbl_size = sizeof(g_esps_queue);
            break;
        case ICF_SYSTEM_TYPE_SIL_EGSE:
            table = g_egse_sil_queue;
            *tbl_size = sizeof(g_egse_sil_queue);
            break;
        case ICF_SYSTEM_TYPE_SIL_ESPS:
            table = g_esps_sil_queue;
            *tbl_size = sizeof(g_esps_sil_queue);
            break;
        default:
            table = g_egse_queue;
            *tbl_size = sizeof(g_egse_queue);
    }
    return table;
}

static struct icf_ctrl_port *icf_choose_hw_port_tbl(int system_type, int *tbl_size) {
    struct icf_ctrl_port *table = NULL;

    switch (system_type) {
        case ICF_SYSTEM_TYPE_EGSE:
            table = g_egse_port;
            *tbl_size = sizeof(g_egse_port);
            break;
        case ICF_SYSTEM_TYPE_ESPS:
            table = g_esps_port;
            *tbl_size = sizeof(g_esps_port);
            break;
        case ICF_SYSTEM_TYPE_SIL_EGSE:
            table = g_egse_sil_port;
            *tbl_size = sizeof(g_egse_sil_port);
            break;
        case ICF_SYSTEM_TYPE_SIL_ESPS:
            table = g_esps_sil_port;
            *tbl_size = sizeof(g_esps_sil_port);
            break;
        default:
            table = g_egse_port;
            *tbl_size = sizeof(g_egse_port);
    }
    return table;
}

static int icf_qidx_to_drivers_id(int qidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = NULL;
    int    map_tbl_size;

    which_map_tbl = icf_choose_map_tbl(system_type, &map_tbl_size);
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].driver_id;
}

static int icf_pidx_to_drivers_id(int pidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = NULL;
    int    map_tbl_size;

    which_map_tbl = icf_choose_map_tbl(system_type, &map_tbl_size);
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].hw_port_idx == pidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].driver_id;
}

static int icf_pidx_to_qidx(int pidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = NULL;
    int    map_tbl_size;

    which_map_tbl = icf_choose_map_tbl(system_type, &map_tbl_size);
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].hw_port_idx == pidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].sw_queue;
}

static int icf_qidx_to_pidx(int qidx, int system_type) {
    int idx = 0;
    struct icf_mapping *which_map_tbl = NULL;
    int    map_tbl_size;

    which_map_tbl = icf_choose_map_tbl(system_type, &map_tbl_size);
    while (idx < get_arr_num(map_tbl_size, sizeof(struct icf_mapping))) {
        if (which_map_tbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return which_map_tbl[idx].hw_port_idx;
}

static int icf_pidx_to_tblidx(int pidx, int system_type) {
    int idx = 0;
    struct icf_ctrl_port *which_port_tbl = NULL;
    int port_tbl_size;

    which_port_tbl = icf_choose_hw_port_tbl(system_type, &port_tbl_size);
    while (idx < get_arr_num(port_tbl_size, sizeof(struct icf_ctrl_port))) {
        if (which_port_tbl[idx].hw_port_idx == pidx)
            break;
        idx++;
    }
    return idx;
}

int icf_ctrlblk_init(struct icf_ctrlblk_t* C, int system_type) {
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport;
    struct icf_driver_ops *drv_ops;
    int idx;
    struct icf_ctrl_queue *which_que_tbl = NULL;
    int que_tbl_size;
    struct icf_ctrl_port *which_port_tbl = NULL;
    int port_tbl_size;
    int hw_port;

    C->system_type = system_type;
    which_que_tbl = icf_choose_sw_queue_tbl(C->system_type, &que_tbl_size);
    which_port_tbl = icf_choose_hw_port_tbl(C->system_type, &port_tbl_size);

    for (idx = 0; idx < get_arr_num(que_tbl_size, sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &which_que_tbl[idx];
        hw_port = icf_qidx_to_pidx(ctrlqueue->queue_idx, C->system_type);
        ctrlqueue->port = &which_port_tbl[icf_pidx_to_tblidx(hw_port, C->system_type)];
        rb_init(&ctrlqueue->data_ring, NUM_OF_CELL);
        C->ctrlqueue[ctrlqueue->queue_idx] = ctrlqueue;
    }
    for (idx = 0; idx < get_arr_num(port_tbl_size, sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &which_port_tbl[idx];
        C->ctrlport[ctrlport->hw_port_idx] = ctrlport;
        ctrlport->drv_priv_ops = icf_drivers[icf_pidx_to_drivers_id(ctrlport->hw_port_idx, system_type)];
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

    struct icf_ctrl_queue *which_que_tbl = NULL;
    struct icf_ctrl_port *which_port_tbl = NULL;
    int port_tbl_size;
    int que_tbl_size;

    which_que_tbl = icf_choose_sw_queue_tbl(C->system_type, &que_tbl_size);
    which_port_tbl = icf_choose_hw_port_tbl(C->system_type, &port_tbl_size);

    for (idx = 0; idx < get_arr_num(que_tbl_size, sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &which_que_tbl[idx];
        rb_deinit(&ctrlqueue->data_ring);
        C->ctrlqueue[idx] = NULL;
    }
    for (idx = 0; idx < get_arr_num(port_tbl_size, sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &which_port_tbl[idx];
        drv_ops = ctrlport->drv_priv_ops;
        if (ctrlport->enable == 0)
            continue;
        drv_ops->close_interface(&ctrlport->drv_priv_data);
    }
    return 0;
}

int icf_rx_dequeue(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct ringbuffer_cell_t *rxcell = NULL;
    rxcell = (struct ringbuffer_cell_t *)rb_pop(&ctrlqueue->data_ring);
    if (rxcell == NULL)
        goto empty;
    memcpy(payload, rxcell->l2frame, size);
    icf_free_mem(rxcell->l2frame);
    icf_free_mem(rxcell);
    debug_hex_dump("icf_rx_dequeue", payload, size);
    return 1;
empty:
    return 0;
}

static int icf_dispatch_rx_frame(int system_type, void *rxframe, int hw_port_idx) {
    int qidx = 0;

    if (system_type == ICF_SYSTEM_TYPE_ESPS) {
        qidx = ESPS_GNC_SW_QIDX;
        debug_hex_dump("esps_dispatch", rxframe, 24);
        return qidx;
    }

    if (system_type == ICF_SYSTEM_TYPE_SIL_ESPS) {
        qidx = ESPS_GNC_SW_QIDX;
        debug_hex_dump("esps_dispatch", rxframe, 24);
        return qidx;
    }

    if (system_type == ICF_SYSTEM_TYPE_SIL_EGSE) {
        qidx = EGSE_SIL_DOWNLINK_SW_QIDX;
        debug_hex_dump("egse_sil_dispatch", rxframe, 24);
        return qidx;
    }

    if (system_type == ICF_SYSTEM_TYPE_EGSE) {
        debug_hex_dump("egse_dispatch", rxframe, 24);
        switch (hw_port_idx) {
            case HW_PORT0:
                qidx = fc_can_cmd_dispatch(rxframe);
                break;
            case HW_PORT2:
                qidx = EGSE_RX_RATETBL_X_SW_QIDX;
                break;
            case HW_PORT3:
                qidx = EGSE_RX_RATETBL_Y_SW_QIDX;
                break;
            case HW_PORT4:
                qidx = EGSE_RX_RATETBL_Z_SW_QIDX;
                break;
            default:
                qidx = fc_can_cmd_dispatch(rxframe);
        }
        return qidx;
    }
    fprintf(stderr, "[%s:%d] System type not match !!\n", __FUNCTION__, __LINE__);
    return qidx;
}

static int icf_l2frame_receive_process(struct icf_ctrlblk_t* C, struct icf_driver_ops *drv_ops,
                                       struct icf_ctrl_port *ctrlport, int rx_buff_size) {
    struct ringbuffer_cell_t *rxcell = NULL;
    struct icf_ctrl_queue *ctrlqueue;
    int qidx;
    /*TODO malloc use the static memory*/
    rxcell = icf_alloc_mem(sizeof(struct ringbuffer_cell_t));
    if (rxcell == NULL) {
        fprintf(stderr, "icf_rx_ctrl_job ring cell allocate fail!!\n");
        goto empty;
    }
    rxcell->frame_full_size = rx_buff_size;
    rxcell->l2frame = icf_alloc_mem(rxcell->frame_full_size);
    if (rxcell->l2frame == NULL) {
        fprintf(stderr, "icf_rx_ctrl_job ring cell allocate fail!!\n");
        goto empty;
    }
    if (drv_ops->recv_data(ctrlport->drv_priv_data, (uint8_t *)rxcell->l2frame, rxcell->frame_full_size) < 0)
        goto empty;
    debug_hex_dump("icf_rx_ctrl_job", (uint8_t *)rxcell->l2frame, rxcell->frame_full_size);
    qidx = icf_dispatch_rx_frame(C->system_type, rxcell->l2frame, ctrlport->hw_port_idx);
    if (qidx == EGSE_EMPTY_SW_QIDX)
        goto empty;
    ctrlqueue = C->ctrlqueue[qidx];
    rb_push(&ctrlqueue->data_ring, rxcell);
    return ICF_STATUS_SUCCESS;
empty:
    icf_free_mem(rxcell->l2frame);
    icf_free_mem(rxcell);
    return ICF_STATUS_FAIL;
}

int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int pidx, int rx_buff_size) {
    struct timeval tv;
    int ret;
    int qidx = 0;
    struct icf_ctrl_port *ctrlport = C->ctrlport[pidx];
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    int sock, fd, nfds;
    tv.tv_sec = 0;
    tv.tv_usec = 100;

    switch (ctrlport->dev_type) {
        case CAN_DEVICE_TYPE:
        case RS422_DEVICE_TYPE:
            drv_ops->fd_zero(ctrlport->drv_priv_data);
            drv_ops->fd_set(ctrlport->drv_priv_data);
            ret = drv_ops->select(ctrlport->drv_priv_data, &tv);
            if (ret < 0)
                fprintf(stderr, "select error: %d\n", ret);
            if (drv_ops->fd_isset(ctrlport->drv_priv_data)) {
                if (icf_l2frame_receive_process(C, drv_ops, ctrlport, rx_buff_size) < 0)
                    break;
                debug_print("[%lf] RX Received !!\n", get_curr_time());
            }
            break;
        case ETHERNET_DEVICE_TYPE:
                if (icf_l2frame_receive_process(C, drv_ops, ctrlport, rx_buff_size) < 0)
                    break;
                debug_print("[%lf] RX Ethernet Received !!\n", get_curr_time());
            break;
        default:
            fprintf(stderr, "[icf_rx_ctrl_job] No such RX  device !!!\n");
    }
    return ICF_STATUS_SUCCESS;
}


int icf_tx_direct(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;

    frame_full_size = size;
    if (ctrlport->drv_priv_data == NULL)
        return ICF_STATUS_SUCCESS;
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
    return ICF_STATUS_SUCCESS;
}

int icf_tx_enqueue(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct ringbuffer_cell_t *txcell;
    uint32_t offset = 0;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;

    whichring = &ctrlqueue->data_ring;
    tx_buffer = icf_alloc_mem(size);

    memcpy(tx_buffer, (uint8_t *) payload, size);

    if (whichring) {
        txcell = icf_alloc_mem(sizeof(struct ringbuffer_cell_t));
        txcell->frame_full_size = size;
        txcell->l2frame = tx_buffer;
        rb_push(whichring, txcell);
    }
    return ICF_STATUS_SUCCESS;
}

int icf_tx_dequeue(struct icf_ctrlblk_t* C, int qidx, void *payload) {
    struct ringbuffer_cell_t *txcell = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    whichring = &ctrlqueue->data_ring;
    txcell = (uint8_t *)rb_pop(whichring);
    if (txcell) {
        memcpy(payload, txcell->l2frame, txcell->frame_full_size);
        debug_hex_dump("icf_tx_dequeue", txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
    }
    return ICF_STATUS_SUCCESS;
}

int icf_tx_ctrl_job(struct icf_ctrlblk_t* C, int qidx) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_cell_t *txcell = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    uint32_t out_frame_size;
    uint32_t offset = 0;
    whichring = &ctrlqueue->data_ring;
    txcell = (uint8_t *)rb_pop(whichring);
    if (txcell) {
        out_frame_size = txcell->frame_full_size;
        if (ctrlport->drv_priv_data == NULL) {
            icf_free_mem(txcell->l2frame);
            icf_free_mem(txcell);
            return ICF_STATUS_SUCCESS;
        }
        if (drv_ops->get_header_size) {
            out_frame_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
        }
        tx_buffer = icf_alloc_mem(out_frame_size);

        if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
            drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) txcell->l2frame, txcell->frame_full_size);
            offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
        }
        memcpy(tx_buffer + offset, (uint8_t *) txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
        drv_ops->send_data(ctrlport->drv_priv_data, tx_buffer, out_frame_size);
        debug_hex_dump("icf_tx_ctrl_job", tx_buffer, out_frame_size);
        icf_free_mem(tx_buffer);
    }
    return ICF_STATUS_SUCCESS;
}

void icf_heartbeat(void) {
    char date_buf[80];
    char currentTime[84] = "";
    static struct timespec ts;
    uint32_t milli;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_sec = time(NULL);
    milli = ts.tv_nsec / 1000000;
    strftime(date_buf, (size_t) 20, "%Y/%m/%d,%H:%M:%S", localtime(&ts.tv_sec));
    snprintf(currentTime, sizeof(currentTime), "%s.%03d", date_buf, milli);
    fprintf(stderr, "[%s] sim_time = %f\n", currentTime, exec_get_sim_time());
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
