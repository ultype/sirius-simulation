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
    int ret = 0;
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
    options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout


    // Set the new attributes
    if ((ret = tcsetattr(fd, TCSANOW, &options)) < 0) {
        fprintf(stderr, "failed to set attr: %d, %s\n", fd, strerror(errno));
        errExit("Encounter a Error !!!\n");
    }
    return ret;
}

int rs422_data_send_scatter(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    struct rs422_device_info_t *dev_info = priv_data;
    uint32_t cur_len = 0;
    int sent_len = 0;
    uint8_t *tx_buffer;
    int ret = 0;
    tx_buffer = payload;
    while (cur_len < frame_len) {
        sent_len = write(dev_info->rs422_fd, tx_buffer + cur_len, frame_len - cur_len);
        if (sent_len < 0) {
            fprintf(stderr, "ERROR status %d\n", sent_len);
            goto error;
        }
        cur_len += sent_len;
    }
error:
    return ret;
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

struct icf_driver_ops icf_driver_rs422_ops = {
    .open_interface = rs422_serialport_init,
    .recv_data = NULL,
    .send_data = rs422_data_send_scatter,

    .header_set = rs422_frame_header_set,
    .header_copy = rs422_frame_header_copy,
    .get_header_size = rs422_frame_get_header_size,
    .select = NULL,
    .fd_clr = NULL,
    .fd_isset = NULL,
    .fd_set = NULL,
    .fd_zero = NULL,
    .close_interface = rs422_serialport_deinit,
};

