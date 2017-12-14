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
            rb_put(&C->g_tvc_ring, pframe);
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
        fprintf(stderr, "[%lf] RX CAN Received !!\n", get_curr_time());
        //  hex_dump("gather rx can", rx_buff, RX_CAN_BUFFER_SIZE);
    }
    return 0;
}

void *icf_alloc_mem (size_t size) {
    return malloc(size);
}

void icf_free_mem (void *ptr) {
    if (ptr)
    {
        free(ptr);
        ptr = NULL;
    }

    return ;
}