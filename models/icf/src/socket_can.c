/************************************************************************
PURPOSE: (Received data by CAN bus.)
*************************************************************************/
#include "socket_can.h"
#include "icf_drivers.h"
int socket_can_init(void **priv_data, char *ifname, int netport) {
    int  status = 0, setflag = 0;
    struct can_device_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct can_device_info_t));
    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    dev_info->set = calloc(1, sizeof(fd_set));
    if (dev_info->set == NULL) {
        fprintf(stderr, "[%s:%d] fd_set Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    if ((dev_info->can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        goto error;
    }
    dev_info->header_size = 0;
    strncpy(dev_info->ifr.ifr_name, ifname, sizeof(dev_info->ifr.ifr_name));
    if (ioctl(dev_info->can_fd, SIOCGIFINDEX, &dev_info->ifr) < 0) {
        fprintf(stderr, "open port %s error\n", ifname);
        errExit("interface not exist !!");
    }
    dev_info->addr.can_family  = PF_CAN;
    dev_info->addr.can_ifindex = dev_info->ifr.ifr_ifindex;

    if (bind(dev_info->can_fd, (struct sockaddr *)(&dev_info->addr), sizeof(dev_info->addr)) < 0) {
        perror("Error in socket bind");
        goto error;
    }
    setflag = fcntl(dev_info->can_fd, F_GETFL);
    setflag = setflag | O_NONBLOCK;
    status = fcntl(dev_info->can_fd, F_SETFL, setflag);
    if (status != 0) {
        fprintf(stderr, "fcntl set fail. status: %s\n", strerror(errno));
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    status = setsockopt(dev_info->can_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
    if (status != 0) {
        fprintf(stderr, "setsockopt fail. status: %s\n", strerror(errno));
    }

    if (dev_info) {
        *priv_data = dev_info;
        printf("Using %s to operation\n", dev_info->ifr.ifr_name);
    }
    return 0;
error:

    if (dev_info->set) {
        free(dev_info->set);
        dev_info->set = NULL;
    }

    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return -1;
}

int socket_can_deinit(void **priv_data) {
    struct can_device_info_t *dev_info = *priv_data;
    close(dev_info->can_fd);
    printf("Closing %s \n", dev_info->ifr.ifr_name);
    if (dev_info->set) {
        free(dev_info->set);
        dev_info->set = NULL;
    }
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}


int can_data_recv_gather(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    int rx_nbytes;
    struct can_frame frame;
    uint32_t buf_offset = 0;
    uint32_t rx_count = 0;
    int ret = 0, idx = 0;
    struct can_device_info_t *dev_info = priv_data;
    if ((buff_size >> 3) << 3 != buff_size) {
        fprintf(stderr, "buffer_size must divide by 8.\n");
    }
    rx_count = buff_size >> 3;
    while (rx_count > 0) {  /* TODO This is very dangerous when didn't know exactly RX packet size */
        rx_nbytes = read(dev_info->can_fd, &frame, sizeof(frame));
        if (rx_nbytes > 0) {
            rx_count--;
            if (frame.can_id & CAN_ERR_FLAG) {
                fprintf(stderr, "error frame\n");
                errExit(__FUNCTION__);
            }
            for (idx = 0; idx < CAN_MAX_DLEN; ++idx) {
                rx_buff[buf_offset] = frame.data[idx];
                buf_offset++;
            }
        }
    }
    return ret;
}

int can_data_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    int rx_nbytes;
    struct can_frame frame;
    uint32_t buf_offset = 0;
    int ret = 0, idx = 0;
    struct can_device_info_t *dev_info = priv_data;
    if (buff_size > CAN_MAX_DLEN) {
        fprintf(stderr, "A single CAN frame size is 8.\n");
    }
    rx_nbytes = read(dev_info->can_fd, &frame, sizeof(frame));
    if (rx_nbytes > 0) {
        if (frame.can_id & CAN_ERR_FLAG) {
            fprintf(stderr, "error frame\n");
            errExit(__FUNCTION__);
        }
        for (idx = 0; idx < CAN_MAX_DLEN; ++idx) {
            rx_buff[buf_offset] = frame.data[idx];
            buf_offset++;
        }
        debug_hex_dump("can_data_recv", rx_buff, CAN_MAX_DLEN);
    }
    return ret;
}

int can_frame_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    int rx_nbytes;
    struct can_device_info_t *dev_info = priv_data;
    struct can_frame *pframe = rx_buff;
    if ((rx_nbytes = read(dev_info->can_fd, pframe, buff_size)) < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            rx_nbytes = 0;
    }
    if (rx_nbytes > 0) {
        if (pframe->can_id & CAN_ERR_FLAG) {
            fprintf(stderr, "error frame\n");
            errExit(__FUNCTION__);
        }
    }
    return rx_nbytes;
}

int can_frame_send(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    int tx_nbytes;
    struct can_device_info_t *dev_info = priv_data;
    struct can_frame *pframe = NULL;
    pframe = (struct can_frame *)payload;
    if ((tx_nbytes = write(dev_info->can_fd, pframe, frame_len)) < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            tx_nbytes = 0;
    }
    return tx_nbytes;
}

int socketcan_select(void *priv_data, struct timeval *timeout) {
    int ret = 0;
    struct can_device_info_t *dev_info = priv_data;
    ret = select(dev_info->can_fd + 1, dev_info->set, NULL, NULL, timeout);
    return ret;
}


void socketcan_fd_clr(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    FD_CLR(dev_info->can_fd, dev_info->set);
}
int socketcan_fd_isset(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    int ret = 0;
    ret = FD_ISSET(dev_info->can_fd, dev_info->set);
    return ret;
}
void socketcan_fd_set(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    FD_SET(dev_info->can_fd, dev_info->set);
}
void socketcan_fd_zero(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    FD_ZERO(dev_info->set);
}

uint32_t socketcan_get_header_size(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    return dev_info->header_size;
}

struct icf_driver_ops icf_driver_socketcan_ops = {
    .open_interface = socket_can_init,
    .recv_data = can_frame_recv,
    .send_data = can_frame_send,
    .get_header_size = socketcan_get_header_size,
    .select = socketcan_select,
    .fd_clr = socketcan_fd_clr,
    .fd_isset = socketcan_fd_isset,
    .fd_set = socketcan_fd_set,
    .fd_zero = socketcan_fd_zero,
    .is_server = NULL,
    .close_interface = socket_can_deinit,
};
