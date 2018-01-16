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
    close(dev_info->netsock_fd);
    printf("Closing %s \n", dev_info->ifr.ifr_name);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}

int create_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    int on = 1;
    dev_info->master_set = calloc(1, sizeof(fd_set));
    if (dev_info->master_set == NULL) {
        fprintf(stderr, "[%s:%d] fd_set Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    if ((dev_info->netsock_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
        goto error;
    }

    err = setsockopt(dev_info->netsock_fd, SOL_SOCKET,  SO_REUSEADDR,
                    (char *)&on, sizeof(on));
    if (err < 0) {
        close(dev_info->netsock_fd);
        errExit("[create_server] setsockopt() failed");
    }

    err = ioctl(dev_info->netsock_fd, FIONBIO, (char *)&on);
    if (err < 0) {
        close(dev_info->netsock_fd);
        errExit("[create_server] ioctl() failed");
    }

    memset(&dev_info->addr, 0, sizeof(dev_info->addr));
    dev_info->addr.sin_family = AF_INET;
    dev_info->addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->addr.sin_port = htons(net_port);
    err = bind(dev_info->netsock_fd, (struct sockaddr *)&dev_info->addr, sizeof(dev_info->addr));
    if (err < 0) {
        close(dev_info->netsock_fd);
        errExit("[create_server] bind() failed");
    }

    err = listen(dev_info->netsock_fd, 1);
    if (err < 0) {
        close(dev_info->netsock_fd);
        errExit("[create_server] listen() failed");
    }

    FD_ZERO(dev_info->master_set);
    FD_SET(dev_info->netsock_fd, dev_info->master_set);

    fprintf(stderr, "create_server: Connection %s:%d\n", ifname, net_port);
    return 0;
error:
    if (dev_info->master_set) {
        free(dev_info->master_set);
        dev_info->master_set = NULL;
    }
    return -1;
}


int create_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    if ((dev_info->netsock_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->addr.sin_family = PF_INET;
    dev_info->addr.sin_addr.s_addr = inet_addr(ifname);
    dev_info->addr.sin_port = htons(net_port);

    err = connect(dev_info->netsock_fd, (struct sockaddr *)&dev_info->addr, sizeof(dev_info->addr));
    if (err == -1) {
        fprintf(stderr, "create_client: Connection error !!\n");
    } else {
        fprintf(stderr, "create_client: Connection %s:%d\n", ifname, net_port);
    }
    return 0;
}

int ethernet_data_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    do {
        rdlen = recv(dev_info->netsock_fd, rx_buff + offset, buff_size - offset, 0);
        offset += rdlen;
    } while (offset < buff_size && rdlen > 0);
    return offset;
}

int ethernet_data_send(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    while (offset < frame_len) {
        wdlen = send(dev_info->netsock_fd, payload + offset, frame_len - offset, 0);
        if (wdlen < 0) {
            //  fprintf(stderr, "[%s:%d] send error: %d\n", __FUNCTION__, __LINE__, wdlen);
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

int ethernet_select(void *priv_data, struct timeval *timeout) {
    int ret = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    ret = select(dev_info->netsock_fd + 1, dev_info->master_set, NULL, NULL, timeout);
    return ret;
}


void ethernet_fd_clr(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_CLR(dev_info->netsock_fd, dev_info->master_set);
}
int ethernet_fd_isset(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    int ret = 0;
    ret = FD_ISSET(dev_info->netsock_fd, dev_info->master_set);
    return ret;
}
void ethernet_fd_set(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_SET(dev_info->netsock_fd, dev_info->master_set);
}

void ethernet_fd_zero(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_ZERO(dev_info->master_set);
}

int ethernet_accept(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return accept(dev_info->netsock_fd, NULL, NULL);
}

struct icf_driver_ops icf_driver_ethernet_ops = {
    .open_interface = ethernet_init,
    .recv_data = ethernet_data_recv,
    .send_data = ethernet_data_send,

    .header_set = NULL,
    .header_copy = NULL,
    .get_header_size = ethernet_get_header_size,

    .select = ethernet_select,
    .fd_clr = ethernet_fd_clr,
    .fd_isset = ethernet_fd_isset,
    .fd_set = ethernet_fd_set,
    .fd_zero = ethernet_fd_zero,
    .close_interface = ethernet_deinit,
};
