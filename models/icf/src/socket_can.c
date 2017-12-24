/************************************************************************
PURPOSE: (Received data by CAN bus.)
*************************************************************************/
#ifdef __cplusplus
    extern "C" {
#endif
#include "socket_can.h"

int socket_can_init(void *priv_data, const char *ifname) {

    int  status = 0, setflag = 0;
    struct can_device_info_t *dev_info = NULL;
    dev_info = calloc(1, sizeof(can_device_info_t));
    if (dev_info == NULL)
    {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }
    if ((dev_info->can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        goto error;
    }
    strncpy(dev_info->ifr.ifr_name, ifname, sizeof(dev_info->ifr.ifr_name));
    if (ioctl(dev_info->can_fd, SIOCGIFINDEX, &dev_info->ifr) < 0) {
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
    printf("Using %s to read\n", dev_info->ifr.ifr_name);
    priv_data = dev_info;
    return 0;
error:
    if (dev_info) {
        free(dev_info);
        dev_info = NULL
    }
    priv_data = NULL;
    return -1;
}

int socket_can_deinit(void *priv_data) {
    struct can_device_info_t *dev_info = priv_data;
    close(dev_info->can_fd);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL
    }
    printf("Closeing %s \n", dev_info->ifr.ifr_name);
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
                exit(EXIT_FAILURE);
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
            exit(EXIT_FAILURE);
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
    rx_nbytes = read(dev_info->can_fd, pframe, buff_size));
    if (rx_nbytes > 0) {
        if (pframe->can_id & CAN_ERR_FLAG) {
            fprintf(stderr, "error frame\n");
            exit(EXIT_FAILURE);
        }
    }
    return rx_nbytes;
}

struct icf_driver_ops icf_driver_socketcan_ops {
    /** Name of the driver interface */
    .name = "socket_can";
    /** One line description of the driver interface */
    .desc = "CAN Driver";
    .open_interface = socket_can_init;
    .recv_data = can_frame_recv;
    .send_data = NULL;
    .send_directly = NULL;
    .close_interface = socket_can_deinit;
};


#ifdef __cplusplus
    }
#endif
