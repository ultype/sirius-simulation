#include "icf_trx_ctrl.h"

static int icf_port_queue_mapping[ICF_TOTAL_EGSE_PORT]{
    TVC_DEVICE_QIDX,
    IMU01_DEVICE_QIDX,
    RATETBL_X_DEVICE_QIDX,
    RATETBL_Y_DEVICE_QIDX,
    RATETBL_Z_DEVICE_QIDX,
    IMU02_DEVICE_QIDX,
    GPSR01_DEVICE_QIDX,
    GPSR02_DEVICE_QIDX,
    GPSRF_EMULATOR_DEVICE_QIDX,
    FLIGHT_COMPUTER_DEVICE_QIDX
};

static struct icf_ctrl_port_info g_egse_port_info[] {
    {"can0",        CAN_DEVICE_TYPE,      ICF_DIRECTION_RX, TVC_DEVICE_QIDX, icf_drivers[0], NULL},
#if 1
    {"/dev/ttyAP0", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, IMU01_DEVICE_QIDX,  NULL, NULL},
    {"/dev/ttyAP1", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, RATETBL_X_DEVICE_QIDX, NULL, NULL},
    {"/dev/ttyAP2", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, RATETBL_Y_DEVICE_QIDX, NULL, NULL},
    {"/dev/ttyAP3", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, RATETBL_Z_DEVICE_QIDX, NULL, NULL},
    {"/dev/ttyAP4", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, IMU02_DEVICE_QIDX,  NULL, NULL},
    {"/dev/ttyAP5", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, GPSR01_DEVICE_QIDX, NULL, NULL},
    {"/dev/ttyAP6", RS422_DEVICE_TYPE,    ICF_DIRECTION_TX, GPSR02_DEVICE_QIDX, NULL, NULL},
    {"empty",       ETHERNET_DEVICE_TYPE, ICF_DIRECTION_TX, GPSRF_EMULATOR_DEVICE_QIDX, NULL, NULL},
    {"empty",       ETHERNET_DEVICE_TYPE, ICF_DIRECTION_TX, FLIGHT_COMPUTER_DEVICE_QIDX, NULL, NULL},
#endif
    {NULL, 0, 0, 0, NULL, NULL}
};


int icf_ctrlblk_init(struct icf_ctrlblk_t* C) {
    struct icf_driver_ops *drv_ops;
    struct icf_ctrl_port_info *port_info;
    int idx;
    for (idx = 0; idx < ICF_TOTAL_EGSE_PORT; ++idx) {
        port_info = &g_egse_port_info[idx];
        C->port_info[idx] = port_info;
        rb_init(&port_info->data_ring, NUM_OF_CELL);
        drv_ops = port_info->drv_priv_ops;
        drv_ops->open_interface(port_info->drv_priv_data, port_info->ifname);
    }
}

int icf_ctrlblk_deinit(struct icf_ctrlblk_t* C) {
    struct icf_driver_ops *drv_ops;
    struct icf_ctrl_port_info *port_info;
    int idx;
    for (idx = 0; idx < ICF_TOTAL_EGSE_PORT; ++idx) {
        port_info = C->port_info[idx];
        rb_deinit(&port_info->data_ring);
        drv_ops->close_interface(port_info->drv_priv_data);
    }
}


int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int port_idx) {
    struct timeval tv;
    //  uint8_t rx_buff[RX_CAN_BUFFER_SIZE] = {0};
    struct can_frame *pframe = NULL;

    fd_set readfds;
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    FD_ZERO(&readfds);
    FD_SET(C->can_info.can_fd, &readfds);
    select(C->can_info.can_fd + 1, &readfds, NULL, NULL, &tv);
    /*TODO malloc need use the static memory*/
    if (FD_ISSET(C->can_info.can_fd, &readfds)) {
        FTRACE_TIME_STAMP(512);
        //  can_data_recv_gather(C->can_info.can_fd, rx_buff, RX_CAN_BUFFER_SIZE);
        pframe = NULL;
        pframe = icf_alloc_mem(sizeof(struct can_frame));
        if (pframe == NULL) {
            fprintf(stderr, "producer frame allocate fail!!\n");
        }
        if (can_frame_recv(C->can_info.can_fd, pframe) > 0) {
            rb_push(&C->g_tvc_ring, pframe);
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
          debug_hex_dump("icf_rx_ctrl_job", (uint8_t *)pframe, RX_CAN_BUFFER_SIZE);
    }
    return 0;
}
#if 1
int icf_tx_direct(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size) {
    
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;

    FTRACE_TIME_STAMP(511);
    frame_full_size = size + (CONFIG_EGSE_CRC_HEADER_ENABLE ? RS422_HEADER_SIZE : 0);
    tx_buffer = icf_alloc_mem(frame_full_size);
#if CONFIG_EGSE_CRC_HEADER_ENABLE
    rs422_frame_header_set(&C->rs422_info[qidx].frame, (uint8_t *) payload, size);
    offset += rs422_frame_header_copy(tx_buffer, &C->rs422_info[qidx].frame);
#endif /* CONFIG_EGSE_CRC_HEADER_ENABLE */
    rs422_frame_payload_copy(tx_buffer + offset, (uint8_t *) payload, size);
    rs422_data_send_scatter(C->rs422_info[qidx].rs422_fd, tx_buffer, frame_full_size);
    debug_hex_dump("icf_tx_direct", tx_buffer, frame_full_size);
    icf_free_mem(tx_buffer);
    return 0;
}

int icf_tx_send2ring(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    struct ringbuffer_t *whichring = NULL;
    struct ringbuffer_cell_t *txcell;
    uint32_t frame_full_size;
    uint32_t offset = 0;

    whichring = &C->icf_tx_ring[qidx];
    frame_full_size = size + (CONFIG_EGSE_CRC_HEADER_ENABLE ? RS422_HEADER_SIZE : 0);
    tx_buffer = icf_alloc_mem(frame_full_size);
#if CONFIG_EGSE_CRC_HEADER_ENABLE
    rs422_frame_header_set(&C->rs422_info[qidx].frame, (uint8_t *) payload, size);
    offset += rs422_frame_header_copy(tx_buffer, &C->rs422_info[qidx].frame);
#endif /* CONFIG_EGSE_CRC_HEADER_ENABLE */
    rs422_frame_payload_copy(tx_buffer + offset, (uint8_t *) payload, size);

    if(whichring) {
        txcell = icf_alloc_mem(sizeof(struct ringbuffer_cell_t));
        txcell->frame_full_size = frame_full_size;
        txcell->l2frame = tx_buffer;
        rb_push(whichring, txcell);
    }

    return 0;
}

int icf_tx_ctrl_job(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx) {
    struct ringbuffer_cell_t *txcell = NULL;
    struct ringbuffer_t *whichring = NULL;
    whichring = &C->icf_tx_ring[qidx];
    txcell = (uint8_t *)rb_pop(whichring);
    if(txcell) {
        rs422_data_send_scatter(C->rs422_info[qidx].rs422_fd, (uint8_t *)txcell->l2frame, txcell->frame_full_size);
        debug_hex_dump("icf_tx_ctrl_job", txcell->l2frame, txcell->frame_full_size);
        icf_free_mem(txcell->l2frame);
        icf_free_mem(txcell);
    }

    return 0;
}

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
