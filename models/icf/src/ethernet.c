#include "ethernet.h"


int ethernet_init(void **priv_data, char *ifname, int netport) {
    struct ethernet_device_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct ethernet_device_info_t));
    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    if (create_client(dev_info, ifname, netport) < 0) {
        errExit("ethernet_init :Error create client");
    }
    dev_info->header_size = 0;
    *priv_data = dev_info;
    return 0;

error:
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return -1;
}

int ethernet_deinit(void **priv_data) {
    struct ethernet_device_info_t *dev_info = *priv_data;
    close(dev_info->client_fd);
    printf("Closing %s \n", dev_info->ifr.ifr_name);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}

int create_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    if ((dev_info->client_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->addr.sin_family = PF_INET;
    dev_info->addr.sin_addr.s_addr = inet_addr(ifname);
    dev_info->addr.sin_port = htons(net_port);

    err = connect(dev_info->client_fd, (struct sockaddr *)&dev_info->addr, sizeof(dev_info->addr));
    if (err == -1) {
        fprintf(stderr, "create_client: Connection error !!\n");
        errExit(__FUNCTION__);
    } else {
        fprintf(stderr, "create_client: Connection %s:%d\n", ifname, net_port);
    }
    return 0;
}

int create_server(struct ethernet_device_info_t *dev_info) {
}

int ethernet_data_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    do {
        rdlen = recv(dev_info->client_fd, rx_buff + offset, buff_size - offset, 0);
        offset += rdlen;
    } while (offset < buff_size && rdlen > 0);
    return offset;
}

int ethernet_data_send(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    while (offset < frame_len) {
        wdlen = send(dev_info->client_fd, payload + offset, frame_len - offset, 0);
        if (wdlen < 0) {
            fprintf(stderr, "[%s:%d] send error: %d\n", __FUNCTION__, __LINE__, wdlen);
            break;
        }
        offset += wdlen;
    }
    return offset;
}

uint32_t ethernet_get_header_size(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return dev_info->header_size;
}

struct icf_driver_ops icf_driver_ethernet_ops = {
    .open_interface = ethernet_init,
    .recv_data = NULL,
    .send_data = ethernet_data_send,

    .header_set = NULL,
    .header_copy = NULL,
    .get_header_size = ethernet_get_header_size,

    .select = NULL,
    .fd_clr = NULL,
    .fd_isset = NULL,
    .fd_set = NULL,
    .fd_zero = NULL,
    .close_interface = ethernet_deinit,
};
