#ifndef MODELS_ICF_INCLUDE_SOCKET_CAN_H_
#define MODELS_ICF_INCLUDE_SOCKET_CAN_H_

#include "icf_utility.h"

#define CAN_MAX_DLEN  8
#define RX_CAN_BUFFER_SIZE ((CONFIG_EGSE_CRC_HEADER_ENABLE ? (56) : (8)))

struct can_device_info_t {
    int can_fd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    fd_set *set;
};


#ifdef __cplusplus
extern "C" {
#endif
int socket_can_init(void **priv_data, char *ifname, int netport);
int can_data_recv_gather(void *priv_data, uint8_t *rx_buff, uint32_t buff_size);
int can_data_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size);
int can_frame_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size);
#ifdef __cplusplus
}
#endif
#endif   // MODELS_ICF_INCLUDE_SOCKET_CAN_H_
