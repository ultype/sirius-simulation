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
    close(dev_info->client_fd);
    printf("Closing %s \n", dev_info->ifr.ifr_name);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}

static int ethernet_create_socket_server(struct ethernet_device_info_t *dev_info, int net_port) {
    int err;
    int optval = 1; /* prevent from address being taken */
    if ((dev_info->server_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
    }
    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    memset(&dev_info->server_addr, 0, sizeof(dev_info->server_addr));
    dev_info->server_addr.sin_family = AF_INET;
    dev_info->server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->server_addr.sin_port = htons(net_port);

    err = setsockopt(dev_info->server_fd, SOL_SOCKET,  SO_REUSEADDR, (char *)&optval, sizeof(int));
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[create_server] setsockopt() failed");
    }

    err = bind(dev_info->server_fd, (struct sockaddr *)&dev_info->server_addr, sizeof(dev_info->server_addr));
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[create_server] bind() failed");
    }

    err = listen(dev_info->server_fd, 5);
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[create_server] listen() failed");
    }
    return dev_info->server_fd;
}


int create_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 1;

    dev_info->server_fd = ethernet_create_socket_server(dev_info, net_port);
    if (dev_info->server_fd < 0) {
        fprintf(stderr, "[%s:%d] Socket create fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    dev_info->client_fd = accept(dev_info->server_fd,(struct sockaddr*) &(dev_info->client_addr), &(dev_info->client_addr_len));
    if (dev_info->client_fd < 0) {
        fprintf(stderr, "create_server: Accept Fail ... %s:%d\n", ifname, net_port);
        goto error;
    }
    fprintf(stderr, "create_server: Accept on ... %s:%d\n", ifname, net_port);
    return 0;
error:
    return -1;
}


int create_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 0;
    if ((dev_info->client_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->client_addr.sin_family = PF_INET;
    dev_info->client_addr.sin_addr.s_addr = inet_addr(ifname);
    dev_info->client_addr.sin_port = htons(net_port);

    err = connect(dev_info->client_fd, (struct sockaddr *)&dev_info->client_addr, sizeof(dev_info->client_addr));
    if (err < 0) {
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

int ethernet_is_server(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
   return dev_info->server_enable;
}
int ethernet_get_client_fd(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return dev_info->client_fd;
}

int ethernet_accept(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    dev_info->client_fd = accept(dev_info->server_fd,(struct sockaddr*) &(dev_info->client_addr), &(dev_info->client_addr_len));
    return dev_info->client_fd;
}

struct icf_driver_ops icf_driver_ethernet_ops = {
    .open_interface = ethernet_init,
    .recv_data = ethernet_data_recv,
    .send_data = ethernet_data_send,

    .header_set = NULL,
    .header_copy = NULL,
    .get_header_size = ethernet_get_header_size,
    .is_server = ethernet_is_server,
    .get_client_fd = ethernet_get_client_fd,
    .accept = ethernet_accept,
    .close_interface = ethernet_deinit,
};
