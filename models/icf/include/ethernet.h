#ifndef MODELS_ICF_INCLUDE_ETHERNET_H_
#define MODELS_ICF_INCLUDE_ETHERNET_H_
#include "icf_utility.h"
#include "icf_drivers.h"
struct ethernet_device_info_t {
    char ifname[IFNAMSIZ];
    int server_enable;
    int netsock_fd;
    fd_set *active_fdset;
    fd_set *read_fdset;
    int nfds;
    struct ifreq ifr;
    struct sockaddr_in addr;
    uint32_t header_size;
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
