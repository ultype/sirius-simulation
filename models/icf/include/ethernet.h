#ifndef MODELS_ICF_INCLUDE_ETHERNET_H_
#define MODELS_ICF_INCLUDE_ETHERNET_H_
#include "icf_utility.h"
#include "icf_drivers.h"
struct ethernet_device_info_t {
    char ifname[IFNAMSIZ];
    int server_enable;
    struct ifreq ifr;
    int server_fd;
    int client_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int client_addr_len;
    uint32_t header_size;
    int sock_type;
};

#ifdef __cplusplus
extern "C" {
#endif
int ethernet_init(void **priv_data, char *ifname, int netport);
int ethernet_deinit(void **priv_data);
#ifdef __cplusplus
}
#endif
#endif  // MODELS_ICF_INCLUDE_ETHERNET_H_
