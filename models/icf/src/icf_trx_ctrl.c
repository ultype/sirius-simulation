#include "icf_trx_ctrl.h"

static const struct icf_mapping g_icf_maptbl[] = {
    {TVC_SW_QIDX,       HW_PORT0, ICF_DRIVERS_ID0},
    {IMU01_SW_QIDX,     HW_PORT1, ICF_DRIVERS_ID1},
    {RATETBL_X_SW_QIDX, HW_PORT2, ICF_DRIVERS_ID1},
    {RATETBL_Y_SW_QIDX, HW_PORT3, ICF_DRIVERS_ID1},
    {RATETBL_Z_SW_QIDX, HW_PORT4, ICF_DRIVERS_ID1},
    {IMU02_SW_QIDX,     HW_PORT5, ICF_DRIVERS_ID1},
    {GPSR01_SW_QIDX,    HW_PORT6, ICF_DRIVERS_ID1},
    {GPSR02_SW_QIDX,    HW_PORT7, ICF_DRIVERS_ID1}
};


static struct icf_ctrl_port g_egse_port[] = {
    {HW_PORT0, "can0",         CAN_DEVICE_TYPE,    NULL, NULL},
    {HW_PORT1, "/dev/ttyAP0",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT2, "/dev/ttyAP1",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT3, "/dev/ttyAP2",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT4, "/dev/ttyAP3",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT5, "/dev/ttyAP4",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT6, "/dev/ttyAP5",  RS422_DEVICE_TYPE,  NULL, NULL},
    {HW_PORT7, "/dev/ttyAP6",  RS422_DEVICE_TYPE,  NULL, NULL}
};

static struct icf_ctrl_queue g_egse_queue[] = {
    {TVC_SW_QIDX,         ICF_DIRECTION_RX, &g_egse_port[HW_PORT0]},
    {IMU01_SW_QIDX,       ICF_DIRECTION_TX, &g_egse_port[HW_PORT1]},
    {RATETBL_X_SW_QIDX,   ICF_DIRECTION_TX, &g_egse_port[HW_PORT2]},
    {RATETBL_Y_SW_QIDX,   ICF_DIRECTION_TX, &g_egse_port[HW_PORT3]},
    {RATETBL_Z_SW_QIDX,   ICF_DIRECTION_TX, &g_egse_port[HW_PORT4]},
    {IMU02_SW_QIDX,       ICF_DIRECTION_TX, &g_egse_port[HW_PORT5]},
    {GPSR01_SW_QIDX,      ICF_DIRECTION_TX, &g_egse_port[HW_PORT6]},
    {GPSR02_SW_QIDX,      ICF_DIRECTION_TX, &g_egse_port[HW_PORT7]}
};

int icf_qidx_to_drivers_id(int qidx) {
    int idx = 0;
    while(idx < get_arr_num(sizeof(g_icf_maptbl), sizeof(struct icf_mapping))) {
        if (g_icf_maptbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return g_icf_maptbl[idx].driver_id;
}

int icf_pidx_to_drivers_id(int pidx) {
    int idx = 0;
    while(idx < get_arr_num(sizeof(g_icf_maptbl), sizeof(struct icf_mapping))) {
        if (g_icf_maptbl[idx].hw_port == pidx)
            break;
        idx++;
    }
    return g_icf_maptbl[idx].driver_id;
}

int icf_pidx_to_qidx(int pidx) {
    int idx = 0;
    while(idx < get_arr_num(sizeof(g_icf_maptbl), sizeof(struct icf_mapping))) {
        if (g_icf_maptbl[idx].hw_port == pidx)
            break;
        idx++;
    }
    return g_icf_maptbl[idx].sw_queue;
}

int icf_qidx_to_pidx (int qidx) {
    int idx = 0;
    while(idx < get_arr_num(sizeof(g_icf_maptbl), sizeof(struct icf_mapping))) {
        if (g_icf_maptbl[idx].sw_queue == qidx)
            break;
        idx++;
    }
    return g_icf_maptbl[idx].hw_port;
}

int icf_ctrlblk_init(struct icf_ctrlblk_t* C) {
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport;
    struct icf_driver_ops *drv_ops;
    int idx;
    for (idx = 0; idx < get_arr_num(sizeof(g_egse_queue), sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &g_egse_queue[idx];
        rb_init(&ctrlqueue->data_ring, NUM_OF_CELL);
        C->ctrlqueue[idx] = ctrlqueue;
    }
    for (idx = 0; idx < get_arr_num(sizeof(g_egse_port), sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &g_egse_port[idx];
        ctrlport->drv_priv_ops = icf_drivers[icf_pidx_to_drivers_id(idx)];
        drv_ops = ctrlport->drv_priv_ops;
        if (idx == 5 || idx == 6)
            continue;
        drv_ops->open_interface(&ctrlport->drv_priv_data, ctrlport->ifname);
    }
    return 0;
}

int icf_ctrlblk_deinit(struct icf_ctrlblk_t* C) {
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport;
    struct icf_driver_ops *drv_ops;
    int idx;
    for (idx = 0; idx < get_arr_num(sizeof(g_egse_queue), sizeof(struct icf_ctrl_queue)); idx++) {
        ctrlqueue = &g_egse_queue[idx];
        rb_deinit(&ctrlqueue->data_ring);
        C->ctrlqueue[idx] = NULL;
    }
    for (idx = 0; idx < get_arr_num(sizeof(g_egse_port), sizeof(struct icf_ctrl_port)); idx++) {
        ctrlport = &g_egse_port[idx];
        drv_ops = ctrlport->drv_priv_ops;
        if (idx == 5 || idx == 6)
            continue;
        drv_ops->close_interface(&ctrlport->drv_priv_data);
    }
    return 0;
}


int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int pidx) {
    struct timeval tv;
    int ret;
    struct icf_ctrl_queue *ctrlqueue;
    struct icf_ctrl_port *ctrlport = &g_egse_port[pidx];
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
        FTRACE_TIME_STAMP(512);
        //  can_data_recv_gather(C->can_info.can_fd, rx_buff, RX_CAN_BUFFER_SIZE);
        pframe = NULL;
        pframe = icf_alloc_mem(sizeof(struct can_frame));
        if (pframe == NULL) {
            fprintf(stderr, "producer frame allocate fail!!\n");
        }
        if (drv_ops->recv_data(ctrlport->drv_priv_data, (uint8_t *)pframe, sizeof(struct can_frame)) > 0) {
            ctrlqueue = C->ctrlqueue[TVC_SW_QIDX];
            rb_push(&ctrlqueue->data_ring, pframe);
        }
#if 0  // CONFIG_EGSE_CRC_HEADER_ENABLE
        struct esps2egse_header_t *rx_header;
        rx_header = (struct esps2egse_header_t *)rx_buff;
        if (crc_checker(rx_header->crc, (rx_buff + sizeof(struct esps2egse_header_t)),
                        rx_header->payload_len) == 0) {
            fprintf(stderr, "[%lf] CRC ERROR !!!!\n", get_curr_time());
            exit(EXIT_FAILURE);
        }
#endif /* CONFIG_EGSE_CRC_HEADER_ENABLE */
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

    FTRACE_TIME_STAMP(511);
    frame_full_size = size + (CONFIG_EGSE_CRC_HEADER_ENABLE ? 
                              (drv_ops->get_header_size(ctrlport->drv_priv_data)) : 0);
    tx_buffer = icf_alloc_mem(frame_full_size);
#if CONFIG_EGSE_CRC_HEADER_ENABLE
        drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload, size);
        offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
#endif /* CONFIG_EGSE_CRC_HEADER_ENABLE */
    memcpy(tx_buffer + offset, (uint8_t *) payload, size);
    drv_ops->send_data(ctrlport->drv_priv_data, tx_buffer, frame_full_size);
    debug_hex_dump("icf_tx_direct", tx_buffer, frame_full_size);
    icf_free_mem(tx_buffer);
    return 0;
}

int icf_tx_send2ring(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct ringbuffer_cell_t *txcell;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;

    whichring = &ctrlqueue->data_ring;
    frame_full_size = size + (CONFIG_EGSE_CRC_HEADER_ENABLE ? 
                            (drv_ops->get_header_size(ctrlport->drv_priv_data)) : 0);
    tx_buffer = icf_alloc_mem(frame_full_size);
#if CONFIG_EGSE_CRC_HEADER_ENABLE
        drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload, size);
        offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
#endif /* CONFIG_EGSE_CRC_HEADER_ENABLE */
    memcpy(tx_buffer + offset, (uint8_t *) payload, size);

    if(whichring) {
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
    if(txcell) {
        drv_ops->send_data(ctrlport->drv_priv_data, txcell->l2frame, txcell->frame_full_size);
        debug_hex_dump("icf_tx_ctrl_job", txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
    }

    return 0;
}
#if 0
int icf_eth_tx_direct(struct icf_tx_ctrl_t* C, void *payload, uint32_t size) {
    
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;

    FTRACE_TIME_STAMP(511);
    frame_full_size = size;
    tx_buffer = icf_alloc_mem(frame_full_size);
    memcpy(tx_buffer + offset, (uint8_t *) payload, size);
    ethernet_data_send(C->eth_info.client_fd, tx_buffer, frame_full_size);
    debug_hex_dump("icf_eth_tx_direct", tx_buffer, frame_full_size);
    icf_free_mem(tx_buffer);
    return 0;
}

int icf_eth_tx_enqueue2ring(struct icf_tx_ctrl_t* C, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct ringbuffer_cell_t *txcell;
    uint32_t frame_full_size;
    uint32_t offset = 0;

    whichring = &C->icf_tx_eth_ring;
    frame_full_size = size;
    tx_buffer = icf_alloc_mem(frame_full_size);
    memcpy(tx_buffer, (uint8_t *) payload, size);

    if(whichring) {
        txcell = icf_alloc_mem(sizeof(struct ringbuffer_cell_t));
        txcell->frame_full_size = frame_full_size;
        txcell->l2frame = tx_buffer;
        rb_push(whichring, txcell);
    }

    return 0;
}

int icf_eth_tx_ctrl_job(struct icf_tx_ctrl_t* C) {
    struct ringbuffer_cell_t *txcell = NULL;
    struct ringbuffer_t *whichring = NULL;
    whichring = &C->icf_tx_eth_ring;
    txcell = (uint8_t *)rb_pop(whichring);
    if(txcell) {
        ethernet_data_send(C->eth_info.client_fd, (uint8_t *)txcell->l2frame, txcell->frame_full_size);
        debug_hex_dump("icf_eth_tx_ctrl_job", txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
    }

    return 0;
}
#endif


void *icf_alloc_mem(size_t size) {
    return calloc(1,size);
}


void icf_free_mem(void *ptr) {
    if (ptr) {
        free(ptr);
        ptr = NULL;
    }
    return;
}
