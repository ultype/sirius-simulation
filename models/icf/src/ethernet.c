#include "ethernet.h"


int ethernet_init(void **priv_data, char *ifname, int netport) {
    struct ethernet_device_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct ethernet_device_info_t));

    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    if (strstr(ifname, "server")) {
        if (create_server(dev_info, ifname, netport) < 0)
            errExit("ethernet_init :Error create client");
    } else {
        if (create_client(dev_info, ifname, netport) < 0)
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

    if (dev_info->active_fdset) {
        free(dev_info->active_fdset);
        dev_info->active_fdset = NULL;
    }
    if (dev_info->read_fdset) {
        free(dev_info->read_fdset);
        dev_info->read_fdset = NULL;
    }
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}

static int ethernet_create_socket_server(struct ethernet_device_info_t *dev_info, int net_port) {

    int sock = 0;
    int err;
    int optval = 1; /* prevent from address being taken */
    if ((sock = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
    }

    err = setsockopt(sock, SOL_SOCKET,  SO_REUSEADDR, (char *)&optval, sizeof(int));
    if (err < 0) {
        close(sock);
        errExit("[create_server] setsockopt() failed");
    }

    err = ioctl(sock, FIONBIO, (char *)&optval);
    if (err < 0) {
        close(sock);
        errExit("[create_server] ioctl() failed");
    }

    memset(&dev_info->addr, 0, sizeof(dev_info->addr));
    dev_info->addr.sin_family = AF_INET;
    dev_info->addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->addr.sin_port = htons(net_port);
    err = bind(sock, (struct sockaddr *)&dev_info->addr, sizeof(dev_info->addr));
    if (err < 0) {
        close(sock);
        errExit("[create_server] bind() failed");
    }

    err = listen(sock, 1);
    if (err < 0) {
        close(sock);
        errExit("[create_server] listen() failed");
    }
    return sock;
}


int create_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 1;
    dev_info->active_fdset = calloc(1, sizeof(fd_set));
    if (dev_info->active_fdset == NULL) {
        fprintf(stderr, "[%s:%d] fd_set Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    dev_info->read_fdset = calloc(1, sizeof(fd_set));
    if (dev_info->read_fdset == NULL) {
        fprintf(stderr, "[%s:%d] fd_set Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    dev_info->netsock_fd = ethernet_create_socket_server(dev_info, net_port);
    if (dev_info->netsock_fd < 0) {
        fprintf(stderr, "[%s:%d] Socket create fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    dev_info->nfds = dev_info->netsock_fd + 1;

    fprintf(stderr, "create_server: Listen on ... %s:%d\n", ifname, net_port);
    return 0;
error:
    if (dev_info->active_fdset) {
        free(dev_info->active_fdset);
        dev_info->active_fdset = NULL;
    }
    if (dev_info->read_fdset) {
        free(dev_info->read_fdset);
        dev_info->read_fdset = NULL;
    }
    return -1;
}


int create_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 0;
    dev_info->active_fdset = NULL;
    dev_info->read_fdset = NULL;
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
    *(dev_info->read_fdset) = *(dev_info->active_fdset);
    ret = select(dev_info->nfds, dev_info->active_fdset, NULL, NULL, timeout);
    return ret;
}


void ethernet_fd_clr(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_CLR(dev_info->netsock_fd, dev_info->active_fdset);
}
int ethernet_fd_isset(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    int ret = 0;
    ret = FD_ISSET(dev_info->netsock_fd, dev_info->read_fdset);
    return ret;
}

int ethernet_accept(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    int newsock;
    newsock = accept(dev_info->netsock_fd, (struct sockaddr *)&dev_info->addr, sizeof(dev_info->addr));
    if (newsock < 0) {
        fprintf(stderr, "error: failed to accept connection\n");
    }

    if (newsock >= dev_info->nfds) 
        dev_info->nfds = newsock + 1;
    FD_SET(newsock, dev_info->active_fdset);
}

void ethernet_fd_set(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_SET(dev_info->netsock_fd, dev_info->active_fdset);
}

void ethernet_fd_zero(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    FD_ZERO(dev_info->active_fdset);
}

int ethernet_is_server(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
   return dev_info->server_enable;
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
    .accept = ethernet_accept, 
    .is_server = ethernet_is_server,
    .close_interface = ethernet_deinit,
};
