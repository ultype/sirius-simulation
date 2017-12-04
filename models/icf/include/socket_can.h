#ifndef __SOCKET_CAN_H__
#define __SOCKET_CAN_H__

#include "icf_export.h"

#define CAN_MAX_DLEN  8
#define RX_CAN_BUFFER_SIZE ((CONFIG_EGSE_CRC_HEADER_ENABLE ? (56) : (8)))
struct can_device_info_t {
	char ifname[IFNAMSIZ];
	int can_fd;
	struct ifreq ifr;
	struct sockaddr_can addr;
};


#ifdef __cplusplus
extern "C" {
#endif
int socket_can_init(struct can_device_info_t *can_device, const char * ifname, uint32_t name_size);
int can_data_recv_gather(int can_fd, uint8_t *rx_buff, uint32_t buff_size);
int can_data_recv(int can_fd, uint8_t *rx_buff, uint32_t buff_size);
#ifdef __cplusplus
}
#endif
#endif /* __SOCKET_CAN_H__ */