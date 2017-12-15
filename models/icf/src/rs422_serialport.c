/************************************************************************
PURPOSE: (Send data by RS422 serial port.)
*************************************************************************/
#ifdef __cplusplus
    extern "C" {
#endif
#include "rs422_serialport.h"

const char *que_port_map[8] = {
    "/dev/ttyAP0",
    "/dev/ttyAP1",
    "/dev/ttyAP2",
    "/dev/ttyAP3",
    "/dev/ttyAP4",
    "/dev/ttyAP5",
    "/dev/ttyAP6",
    "/dev/ttyAP7"
};



int rs422_devinfo_init(struct rs422_device_info_t *dev_info,
                       const char *portname,
                       uint8_t qidx) {
    strncpy(dev_info->portname, portname, IFNAMSIZ);
    dev_info->qidx = qidx;
    dev_info->frame.crc = 0;
    dev_info->frame.payload_len = 0;
    dev_info->frame.seq_no = 0;
    return 0;
}

int rs422_serialport_init(struct rs422_device_info_t *rs422_dev) {
    struct rs422_device_info_t *dev_info = rs422_dev;
    int  status = 0;
    dev_info->rs422_fd  = open_port(dev_info->portname);
    printf("%s\n", dev_info->portname);
    if (dev_info->rs422_fd  < 0) {
        fprintf(stderr, "open port %s error\n", dev_info->portname);
        exit(EXIT_FAILURE);
    }
    set_interface_attribs(dev_info->rs422_fd, B921600, 0);
    return status;
}

int rs422_serialport_deinit(struct rs422_device_info_t *rs422_dev) {
    int  status = 0;
    close(rs422_dev->rs422_fd);
    return status;
}

int open_port(char *portname) {
    int fd;
    fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Opening %s fail: Error: %d %s\n", portname, errno, strerror(errno));
        exit(EXIT_FAILURE);
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
        exit(EXIT_FAILURE);
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
        exit(EXIT_FAILURE);
    }
    return ret;
}

int32_t rs422_data_send_scatter(int fd, uint8_t *payload, uint32_t frame_len) {
    uint32_t cur_len = 0;
    int sent_len = 0;
    uint8_t *tx_buffer;
    int ret = 0;
    tx_buffer = payload;
    while (cur_len < frame_len) {
        sent_len = write(fd, tx_buffer + cur_len, frame_len - cur_len);
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

int rs422_frame_header_set(struct rs422_frame_header_t *frame, const uint8_t *payload, const uint32_t data_len) {
    int ret = 0;
    frame->payload_len = data_len;
    frame->crc = crc32_create(payload, frame->payload_len);
    frame->seq_no += 1;
    return ret;
}

int rs422_frame_payload_copy(uint8_t *out_buff, const uint8_t *payload, const uint32_t data_len) {
    memcpy(out_buff, payload, data_len);
    return 0;
}

int rs422_frame_header_copy(uint8_t *out_buff, struct rs422_frame_header_t *frame) {
    uint32_t header_offset = 0;
    memcpy(out_buff + header_offset, &frame->payload_len, 4);
    header_offset += 4;

    memcpy(out_buff + header_offset, &frame->crc, 4);
    header_offset += 4;

    memcpy(out_buff + header_offset, &frame->seq_no, 4);
    header_offset += 4;

    return header_offset;
}
#ifdef __cplusplus
    }
#endif
