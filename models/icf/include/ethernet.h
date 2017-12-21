#ifndef MODELS_ICF_INCLUDE_ETHERNET_H_
#define MODELS_ICF_INCLUDE_ETHERNET_H_
#include "icf_utility.h"

struct ethernet_device_info_t {
    char ifname[IFNAMSIZ];
    int client_fd;
    int server_fd;
    struct ifreq ifr;
    struct sockaddr_in addr;
};

#ifdef __cplusplus
extern "C" {
#endif
int ethernet_init(struct ethernet_device_info_t *dev_info);
int ethernet_deinit(struct ethernet_device_info_t *dev_info);
uint32_t ethernet_data_recv(int sockfd, uint8_t *rx_buff, uint32_t buff_size);
uint32_t ethernet_data_send(int sockfd, uint8_t *tx_buff, uint32_t buff_size);
#ifdef __cplusplus
}
#endif
#endif  // MODELS_ICF_INCLUDE_ETHERNET_H_