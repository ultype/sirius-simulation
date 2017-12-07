#include "icf_trx_ctrl.h"

int icf_rx_ctrl_job(struct icf_rx_ctrl_t* C) {
    struct timeval tv;
    uint8_t rx_buff[RX_CAN_BUFFER_SIZE] = {0};

    fd_set readfds;
    tv.tv_sec = 0;
    tv.tv_usec = 100;

    FD_ZERO(&readfds);
    FD_SET(C->can_info.can_fd, &readfds);
    select(C->can_info.can_fd + 1, &readfds, NULL, NULL, &tv);
    if (FD_ISSET(C->can_info.can_fd, &readfds)) {
        FTRACE_TIME_STAMP(512);
        can_data_recv_gather(C->can_info.can_fd, rx_buff, RX_CAN_BUFFER_SIZE);
#if CONFIG_EGSE_CRC_HEADER_ENABLE
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
