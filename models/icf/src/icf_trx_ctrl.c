#include "icf_trx_ctrl.h"

int icf_rx_ctrl_init(struct icf_rx_ctrl_t* C) {
    socket_can_init(&C->can_info, "can0", 4);
    rb_init(&C->g_tvc_ring, NUM_OF_CELL);
}

int icf_rx_ctrl_deinit(struct icf_rx_ctrl_t* C) {
    rb_deinit(&C->g_tvc_ring);
    socket_can_deinit(&C->can_info);
}

int icf_rx_ctrl_job(struct icf_rx_ctrl_t* C) {
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

int icf_tx_ctrl_init(struct icf_tx_ctrl_t* C) {

    uint8_t qidx;
    rs422_devinfo_init(&C->rs422_info[IMU_01], que_port_map[IMU_01], IMU_01);
    rs422_devinfo_init(&C->rs422_info[RATE_TABLE_X], que_port_map[RATE_TABLE_X], RATE_TABLE_X);
    rs422_devinfo_init(&C->rs422_info[RATE_TABLE_Y], que_port_map[RATE_TABLE_Y], RATE_TABLE_Y);
    rs422_devinfo_init(&C->rs422_info[RATE_TABLE_Z], que_port_map[RATE_TABLE_Z], RATE_TABLE_Z);
    rs422_devinfo_init(&C->rs422_info[IMU_02], que_port_map[IMU_02], IMU_02);
    rs422_devinfo_init(&C->rs422_info[GPSR_01], que_port_map[GPSR_01], GPSR_01);
    rs422_devinfo_init(&C->rs422_info[GPSR_02], que_port_map[GPSR_02], GPSR_02);
    
    for (qidx = 0; qidx < RS422_TXQ_NUM; qidx++) {
        if (SERIAL_PORT_IS_ENABLE(qidx)) {
            rs422_serialport_init(&C->rs422_info[qidx]);
            rb_init(&C->icf_tx_ring[qidx], NUM_OF_CELL);
        }
    }
    return 0;
}

int icf_tx_ctrl_deinit(struct icf_tx_ctrl_t* C) {
    uint8_t qidx;
    for (qidx = 0; qidx < RS422_TXQ_NUM; qidx++) {
        if (SERIAL_PORT_IS_ENABLE(qidx))
            rs422_serialport_deinit(&C->rs422_info[qidx]);
    }
    return 0;
}

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
