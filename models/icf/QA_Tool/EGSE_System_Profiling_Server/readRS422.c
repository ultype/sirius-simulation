#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include "rs422_serialport.h"
#include "icf_utility.h"
#include <stdint.h>

#define CONFIG_ESPS_HEADER_ENABLE 0

int rx_frame_receive_with_header(void *dev, uint8_t *rs422payload, int buff_size) {
    struct rs422_device_info_t *dev_info = (struct rs422_device_info_t *)dev;
    int rdlen = 0;
    uint32_t buf_offset = 0;
    memset(rs422payload, 0, buff_size);
    while (buf_offset < dev_info->header_size) {
        rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->header_size - buf_offset);
        if (rdlen < 0)
            fprintf(stderr, "[%s:%d] %d: %s\n", __FUNCTION__, __LINE__, rdlen, strerror(errno));
        buf_offset += rdlen;
    }

    if (buf_offset == dev_info->header_size) {
        memcpy(&dev_info->frame, rs422payload, dev_info->header_size);
        while (buf_offset < dev_info->frame.payload_len + dev_info->header_size) {
            rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->frame.payload_len + dev_info->header_size - buf_offset);
            if (rdlen < 0)
                fprintf(stderr, "[%s:%d] %d: %s\n", __FUNCTION__, __LINE__, rdlen, strerror(errno));
            buf_offset += rdlen;
        }
        hex_dump("RX data payload", (uint8_t *)rs422payload + dev_info->header_size, dev_info->frame.payload_len);
    }
    return rdlen;
}

int rx_frame_receive_without_header(void *dev, uint8_t *rs422payload, int pkt_size) {
    struct rs422_device_info_t *dev_info = (struct rs422_device_info_t *)dev;
    int rdlen = 0;
    uint32_t buf_offset = 0;
    memset(rs422payload, 0, pkt_size);
    while (buf_offset <  pkt_size){
            rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, pkt_size - buf_offset);
            if (rdlen < 0)
                fprintf(stderr, "[%s:%d] %d: %s\n", __FUNCTION__, __LINE__, rdlen, strerror(errno));
            buf_offset += rdlen;
        }
        hex_dump("RX data payload", (uint8_t *)rs422payload, pkt_size);
    return rdlen;
}

int main(int argc, char *argv[]) {
    char portname[IFNAMSIZ] = "/dev/ttyAP0";
    struct rs422_device_info_t *dev_info;
    uint8_t rx_buf[1024] = {0};
    struct timeval timeout;
    fd_set readfs;
    /* Initialize the timeout structure */
    timeout.tv_sec  = 0;
    timeout.tv_usec = 100;

	
	if (argc != 1) {
		printf("%s\n", argv[1]);
        memcpy(portname, argv[1], IFNAMSIZ);
	}
	rs422_serialport_init((void **)&dev_info, portname, CONFIG_ESPS_HEADER_ENABLE);

	/* simple noncanonical input */
	do {
        FD_ZERO(&readfs);
        FD_SET(dev_info->rs422_fd, &readfs);
        if (select(dev_info->rs422_fd + 1, &readfs, NULL, NULL, &timeout) < 0)
            fprintf(stderr, "[%s:%d] %s\n", __FUNCTION__, __LINE__, strerror(errno));
        if (FD_ISSET(dev_info->rs422_fd, &readfs)) {
            fprintf(stderr, "[Time: %f] \n", get_curr_time());
            if (dev_info->header_size > 0) {
                rx_frame_receive_with_header(dev_info, rx_buf, 1024);
            } else {
                rx_frame_receive_without_header(dev_info, rx_buf, 4);
            }
        }
	} while (1);
	close(dev_info->rs422_fd);
	return 0;
}
