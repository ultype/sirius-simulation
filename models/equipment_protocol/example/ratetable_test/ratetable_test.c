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

int rx_frame_receive(void *dev, uint8_t *rs422payload, int pkt_size) {
    struct rs422_device_info_t *dev_info = (struct rs422_device_info_t *)dev;
    int rdlen = 0;
    uint32_t buf_offset = 0;
    memset(rs422payload, 0, pkt_size);
    while (buf_offset < pkt_size) {        
        if ((rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, pkt_size - buf_offset)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                rdlen = 0;                
            } else {
                fprintf(stderr, "%s: %s\n", __FUNCTION__, strerror(errno));
                return -1;
            }
        } else if (rdlen == 0) {
            break;
        }
        buf_offset += rdlen;
    }
    //  hex_dump("RX data payload", (uint8_t *)rs422payload, pkt_size);
    return rdlen;
}

int tx_frame_sendto(void *dev, uint8_t *rs422payload, int pkt_size) {
    struct rs422_device_info_t *dev_info = (struct rs422_device_info_t *)dev;
    int wdlen = 0;
    uint32_t buf_offset = 0;
    while (buf_offset < pkt_size) {
        if ((wdlen = write(dev_info->rs422_fd, rs422payload + buf_offset, pkt_size - buf_offset)) < 0) {
            if (wdlen < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                wdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __FUNCTION__, strerror(errno));
                return -1;
            }
        }
        buf_offset += wdlen;
    }
    hex_dump("TX data payload", (uint8_t *)rs422payload, pkt_size);
    return wdlen;
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
    rs422_serialport_init((void **) &dev_info, portname, 0);


    /* simple noncanonical input */
    do {
        /* Initialize the input set */
        FD_ZERO(&readfs);
        FD_SET(dev_info->rs422_fd, &readfs);
        if (select(dev_info->rs422_fd + 1, &readfs, NULL, NULL, &timeout) < 0)
            fprintf(stderr, "[%s:%d] %s\n", __FUNCTION__, __LINE__, strerror(errno));
        if (FD_ISSET(dev_info->rs422_fd, &readfs)) {
            fprintf(stderr, "[Time: %f] \n", get_curr_time());
            rx_frame_receive(dev_info, rx_buf, 4);
            tx_frame_sendto(dev_info, rx_buf, 4);
        }

    /* repeat read to get full message */
    } while (1);
    close(dev_info->rs422_fd);
    return 0;
}
