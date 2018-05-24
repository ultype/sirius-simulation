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
#include "gpsr_s_nav_tlm.h"


void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

void gpsr_tlm_dump(struct gpsr_s_nav_tlm_frame_t *tlm) {
    fprintf(stderr, "$TLM, %d, %d, %f, %f, %f, %f, %f, %f\n",
            tlm->gps_week_num, tlm->gps_time,
            tlm->posx, tlm->posy, tlm->posz,
            tlm->velx, tlm->vely, tlm->velz);
}


int main(int argc, char *argv[])
{
    char portname[IFNAMSIZ] = "/dev/ttyAP0";
    uint32_t pkt_cnt = 0;
    struct rs422_device_info_t *dev_info;
    uint8_t rx_buf[1024] = {0};
    uint8_t *rs422payload = rx_buf;
    struct timeval timeout;
    fd_set readfs;
    uint32_t buf_offset = 0;
    int rdlen;
    uint32_t idx = 0;
    struct gpsr_s_nav_tlm_frame_t tlm;
    /* Initialize the timeout structure */
    timeout.tv_sec  = 0;
    timeout.tv_usec = 100;
    if (argc != 1) {
        printf("%s\n", argv[1]);
        memcpy(portname, argv[1], IFNAMSIZ);
    }
    rs422_serialport_init((void **) &dev_info, portname, 1);


    /* simple noncanonical input */
    do {
        idx = 0;
        rdlen = 0;
        buf_offset = 0;
        memset(rs422payload, 0, 1024);
        /* Initialize the input set */
        FD_ZERO(&readfs);
        FD_SET(dev_info->rs422_fd, &readfs);
        if (select(dev_info->rs422_fd + 1, &readfs, NULL, NULL, &timeout) < 0)
            fprintf(stderr, "[%s:%d] %s\n", __FUNCTION__, __LINE__, strerror(errno));;
        if (FD_ISSET(dev_info->rs422_fd, &readfs)) {
            while (buf_offset < dev_info->header_size) {
                rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->header_size - buf_offset);
                buf_offset += rdlen;
                if(idx++ == 10) printf("[%s] pkt_cnt: %d\n" ,portname ,pkt_cnt);
            }

            if (buf_offset == dev_info->header_size) {
                memcpy(&dev_info->frame, rs422payload, dev_info->header_size);
                pkt_cnt++;
                while (buf_offset < dev_info->frame.payload_len + dev_info->header_size) {
                    rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->frame.payload_len + dev_info->header_size - buf_offset);
                    buf_offset += rdlen;
                }
                //  hex_dump("RX data payload", (uint8_t *)rs422payload + dev_info->header_size, dev_info->frame.payload_len);
                memcpy(&tlm, rs422payload + dev_info->header_size, sizeof(struct gpsr_s_nav_tlm_frame_t));
                gpsr_tlm_dump(&tlm);
            } else if (rdlen < 0) {
                fprintf(stderr, "Error from read: %d: %s\n", rdlen, strerror(errno));
            }
        }
    /* repeat read to get full message */
    } while (1);
    close(dev_info->rs422_fd);
    return 0;
}
