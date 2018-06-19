/************************************************************************
PURPOSE: (Send data by RS422 serial port.)
*************************************************************************/
#include "rs422_serialport.h"

int rs422_serialport_init(void **priv_data, char *ifname, int is_header_en) {
    struct rs422_device_info_t *dev_info = NULL;
    int  status = 0;
    dev_info = malloc(sizeof(struct rs422_device_info_t));
    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    dev_info->set = calloc(1, sizeof(fd_set));
    if (dev_info->set == NULL) {
        fprintf(stderr, "[%s:%d] fd_set Memory allocate fail. status: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        goto error;
    }

    strncpy(dev_info->portname, ifname, IFNAMSIZ);
    dev_info->frame.crc = 0;
    dev_info->frame.payload_len = 0;
    dev_info->frame.seq_no = 0;
    dev_info->header_size = is_header_en ? sizeof(struct rs422_frame_header_t) : 0;
    dev_info->rs422_fd  = open_port(dev_info->portname);
    printf("%s\n", dev_info->portname);
    if (dev_info->rs422_fd  < 0) {
        fprintf(stderr, "open port %s error\n", dev_info->portname);
        errExit("Encounter a Error !!!\n");
    }
    set_interface_attribs(dev_info->rs422_fd, B230400, 0);

    *priv_data = dev_info;
    return status;
error:
    status = -1;
    if (dev_info->set) {
        free(dev_info->set);
        dev_info->set = NULL;
    }
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return status;
}

int rs422_serialport_deinit(void **priv_data) {
    int  status = 0;
    struct rs422_device_info_t *dev_info = *priv_data;
    close(dev_info->rs422_fd);
    printf("Closing %s \n", dev_info->portname);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
        *priv_data = NULL;
    }
    return status;
}

int open_port(char *portname) {
    int fd;
    fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Opening %s fail: Error: %d %s\n", portname, errno, strerror(errno));
        errExit("Encounter a Error !!!\n");
    } else {
        printf("Using %s to send\n", portname);
    }

    return (fd);
}

int set_interface_attribs(int fd, int speed, int parity) {
    struct termios options;
    int ret = 0, setflag;
    // Get the current options for the port
    if ((ret = tcgetattr(fd, &options)) < 0) {
        fprintf(stderr, "failed to get attr: %d, %s\n", fd, strerror(errno));
        errExit("Encounter a Error !!!\n");
    }

    cfsetospeed(&options, (speed_t)speed);
    cfsetispeed(&options, (speed_t)speed);
    /* Enable the receiver and set local mode...*/
    options.c_cflag |= (CLOCAL | CREAD);

    /* Setting the Character Size */
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~(PARENB | PARODD);
    options.c_cflag |= parity;
    options.c_cflag &= ~CSTOPB;    // 1 stop bits

    /*Setting Hardware Flow Control*/
    options.c_cflag &= ~CRTSCTS;

    /* Local Options: Choosing Raw Input*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* Input Options: Setting Software Flow Control */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL);

    /* Output Options: Choosing Raw Output */
    options.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    options.c_cc[VMIN]  = 0;            // read doesn't block
    options.c_cc[VTIME] = 0.001;            // 0.5 seconds read timeout


    // Set the new attributes
    if ((ret = tcsetattr(fd, TCSANOW, &options)) < 0) {
        fprintf(stderr, "failed to set attr: %d, %s\n", fd, strerror(errno));
        errExit("Encounter a Error !!!\n");
    }

    setflag = fcntl(fd, F_GETFL);
    setflag = setflag | O_NONBLOCK;
    ret = fcntl(fd, F_SETFL, setflag);
    if (ret != 0) {
        fprintf(stderr, "fcntl set fail. ret: %s\n", strerror(errno));
    }
    return ret;
}

int rs422_data_send_scatter(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    struct rs422_device_info_t *dev_info = priv_data;
    uint32_t offset = 0;
    int wdlen = 0;
    uint8_t *tx_buffer;
    tx_buffer = payload;
    while (offset < frame_len) {
        if ((wdlen = write(dev_info->rs422_fd, tx_buffer + offset, frame_len - offset)) < 0) {
            if (wdlen < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                wdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __FUNCTION__, strerror(errno));
                return -1;
            }
        }
        offset += wdlen;
    }
    return offset;
}

int rs422_data_recv_gather(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    struct rs422_device_info_t *dev_info = priv_data;
    uint32_t offset = 0;
    int rdlen = 0;
    uint8_t *rx_buffer;
    rx_buffer = payload;
    memset(rx_buffer, 0, frame_len);
    while (offset < frame_len) {
        if ((rdlen = read(dev_info->rs422_fd, rx_buffer + offset, frame_len - offset)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                rdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __FUNCTION__, strerror(errno));
                return -1;
            }
        } else if (rdlen == 0) {
            break;
        }
        offset += rdlen;
    }
    return offset;
}

uint8_t* rs422_frame_alloc(uint32_t size) {
    return calloc(1, size);
}

void rs422_frame_free(void *ptr) {
    if (ptr) {
        free(ptr);
        ptr = NULL;
    }
    return;
}

int rs422_frame_header_set(void *priv_data, const uint8_t *payload, const uint32_t data_len) {
    int ret = 0;
    struct rs422_device_info_t *dev_info = priv_data;
    dev_info->frame.payload_len = data_len;
    dev_info->frame.crc = crc32_create(payload, dev_info->frame.payload_len);
    dev_info->frame.seq_no += 1;
    return ret;
}

int rs422_frame_payload_copy(uint8_t *out_buff, const uint8_t *payload, const uint32_t data_len) {
    memcpy(out_buff, payload, data_len);
    return 0;
}

int rs422_frame_header_copy(void *priv_data, uint8_t *out_buff) {
    uint32_t header_offset = 0;
    struct rs422_device_info_t *dev_info = priv_data;
    memcpy(out_buff + header_offset, &dev_info->frame.payload_len, 4);
    header_offset += 4;

    memcpy(out_buff + header_offset, &dev_info->frame.crc, 4);
    header_offset += 4;

    memcpy(out_buff + header_offset, &dev_info->frame.seq_no, 4);
    header_offset += 4;

    return header_offset;
}

uint32_t rs422_frame_get_header_size(void *priv_data) {
    struct rs422_device_info_t *dev_info = priv_data;
    return dev_info->header_size;
}

int rs422_select(void *priv_data, struct timeval *timeout) {
    int ret = 0;
    struct rs422_device_info_t *dev_info = priv_data;
    ret = select(dev_info->rs422_fd + 1, dev_info->set, NULL, NULL, timeout);
    return ret;
}


void rs422_fd_clr(void *priv_data) {
    struct rs422_device_info_t *dev_info = priv_data;
    FD_CLR(dev_info->rs422_fd, dev_info->set);
}
int rs422_fd_isset(void *priv_data) {
    struct rs422_device_info_t *dev_info = priv_data;
    int ret = 0;
    ret = FD_ISSET(dev_info->rs422_fd, dev_info->set);
    return ret;
}
void rs422_fd_set(void *priv_data) {
    struct rs422_device_info_t *dev_info = priv_data;
    FD_SET(dev_info->rs422_fd, dev_info->set);
}
void rs422_fd_zero(void *priv_data) {
    struct rs422_device_info_t *dev_info = priv_data;
    FD_ZERO(dev_info->set);
}

struct icf_driver_ops icf_driver_rs422_ops = {
    .open_interface = rs422_serialport_init,
    .recv_data = rs422_data_recv_gather,
    .send_data = rs422_data_send_scatter,

    .header_set = rs422_frame_header_set,
    .header_copy = rs422_frame_header_copy,
    .get_header_size = rs422_frame_get_header_size,
    .select = rs422_select,
    .fd_clr = rs422_fd_clr,
    .fd_isset = rs422_fd_isset,
    .fd_set = rs422_fd_set,
    .fd_zero = rs422_fd_zero,
    .close_interface = rs422_serialport_deinit,
};

