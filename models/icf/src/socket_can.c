/************************************************************************
PURPOSE: (Received data by CAN bus.)
*************************************************************************/
#ifdef __cplusplus
    extern "C" {
#endif
#include "socket_can.h"

int socket_can_init(struct can_device_info_t *can_device, const char * ifname, uint32_t name_size) {
    struct can_device_info_t *dev_info;
    dev_info = can_device;
    int  status = 0, setflag = 0;

    if ((dev_info->can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }
    strncpy(dev_info->ifr.ifr_name, ifname, name_size);
    if (ioctl(dev_info->can_fd, SIOCGIFINDEX, &dev_info->ifr) < 0) {
        errExit("interface not exist !!");
    }
    dev_info->addr.can_family  = PF_CAN;
    dev_info->addr.can_ifindex = dev_info->ifr.ifr_ifindex;

    if (bind(dev_info->can_fd, (struct sockaddr *)(&dev_info->addr), sizeof(dev_info->addr)) < 0) {
        perror("Error in socket bind");
        return -2;
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
    return 0;
}

int socket_can_deinit(struct can_device_info_t *can_device) {
    struct can_device_info_t *dev_info;
    dev_info = can_device;
    close(dev_info->can_fd);
    printf("Closeing %s \n", dev_info->ifr.ifr_name);
    return 0;
}


int can_data_recv_gather(int can_fd, uint8_t *rx_buff, uint32_t buff_size) {
    int rx_nbytes;
    struct can_frame frame;
    uint32_t buf_offset = 0;
    uint32_t rx_count = 0;
    int ret = 0, idx = 0;
    if ((buff_size >> 3) << 3 != buff_size) {
        fprintf(stderr, "buffer_size must divide by 8.\n");
    }
    rx_count = buff_size >> 3;
    while (rx_count > 0) {  /* TODO This is very dangerous when didn't know exactly RX packet size */
        rx_nbytes = read(can_fd, &frame, sizeof(frame));
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

int can_data_recv(int can_fd, uint8_t *rx_buff, uint32_t buff_size) {
    int rx_nbytes;
    struct can_frame frame;
    uint32_t buf_offset = 0;
    int ret = 0, idx = 0;
    if (buff_size > CAN_MAX_DLEN) {
        fprintf(stderr, "A single CAN frame size is 8.\n");
    }
    rx_nbytes = read(can_fd, &frame, sizeof(frame));
    if (rx_nbytes > 0) {
        if (frame.can_id & CAN_ERR_FLAG) {
            fprintf(stderr, "error frame\n");
            exit(EXIT_FAILURE);
        }
        for (idx = 0; idx < CAN_MAX_DLEN; ++idx) {
            rx_buff[buf_offset] = frame.data[idx];
            buf_offset++;
        }
        hex_dump("single rx can", rx_buff, CAN_MAX_DLEN);
    }
    return ret;
}

int can_frame_recv(int can_fd, struct can_frame *pframe) {
    int rx_nbytes;
    rx_nbytes = read(can_fd, pframe, sizeof(struct can_frame));
    if (rx_nbytes > 0) {
        if (pframe->can_id & CAN_ERR_FLAG) {
            fprintf(stderr, "error frame\n");
            exit(EXIT_FAILURE);
        }
        //  hex_dump("CAN RX recive",(uint8_t *)pframe, 16);
    }
    return rx_nbytes;
}
#ifdef __cplusplus
    }
#endif
