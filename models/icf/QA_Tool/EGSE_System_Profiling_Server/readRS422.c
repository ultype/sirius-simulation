#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "rs422_serialport.h"
#include "icf_utility.h"
#include <stdint.h>

#define CFG_GPIO_ENABLE 0

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

#if (CFG_GPIO_ENABLE == 1)
void *gpio_ttl_thread(void *arg)
{
	int *ret = NULL;
	gpio_ttl_init_wrapper();
	return ret;
}
#endif
int main(int argc, char *argv[])
{
	char portname[IFNAMSIZ] = "/dev/ttyAP0";
	
	uint32_t pkt_cnt = 0;
	uint32_t error_cnt = 0;
	struct rs422_device_info_t *dev_info;
	uint8_t rx_buf[1024] = {0};
    uint8_t *rs422payload = rx_buf;

	
	if (argc != 1) {
		printf("%s\n", argv[1]);
        memcpy(portname, argv[1], IFNAMSIZ);
	}
	rs422_serialport_init(&dev_info, portname, 0);

#if (CFG_GPIO_ENABLE == 1)
	int ret;
	pthread_t gpio_thread_id;
	ret = pthread_create(&gpio_thread_id, NULL, gpio_ttl_thread, NULL);
	if (ret) {
		printf("ERROR; return code from gpio_thread_id is %d\n", ret);
		exit(-1);
	}
#endif /* CFG_GPIO_ENABLE */
	/* simple noncanonical input */
	do {
		uint32_t buf_offset = 0;
		int rdlen;
		uint32_t i = 0;
		struct timespec ts;
		memset(rs422payload, 0, 1024);
		
		while (buf_offset < dev_info->header_size) {
			rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->header_size - buf_offset);
			buf_offset += rdlen;
			if(i++ == 10) printf("[%s] pkt_cnt: %d\n" ,portname ,pkt_cnt);
		}

		if (buf_offset == dev_info->header_size) {
			memcpy(&dev_info->frame, rs422payload, dev_info->header_size);
			clock_gettime(CLOCK_MONOTONIC, &ts);
			pkt_cnt++;
			while (buf_offset < dev_info->frame.payload_len + dev_info->header_size) {
				rdlen = read(dev_info->rs422_fd, rs422payload + buf_offset, dev_info->frame.payload_len + dev_info->header_size - buf_offset);
				buf_offset += rdlen;
			}
			//hex_dump("RX data", (uint8_t *)rs422payload, p_data_info->payload_len + dev_info->header_size);
#if CONFIG_ESPS_HEADER_ENABLE
			if(crc_checker(dev_info->frame.crc, (rs422payload + dev_info->header_size),dev_info->frame.payload_len) == 0){
				error_cnt++;
				printf("[%lf: error cnt: %d] SEQ: %d , %s CRC ERROR !!!!\n", get_curr_time(), error_cnt, dev_info->frame.seq_no,portname);
				exit(EXIT_FAILURE);
			} else {
				fprintf(stderr, "[%lf:%d] %s CRC PASS !!!!\n", get_curr_time(), dev_info->frame.seq_no, portname);
			}
#endif /* CONFIG_ESPS_HEADER_ENABLE */
		} else if (rdlen < 0) {
		    fprintf(stderr, "Error from read: %d: %s\n", rdlen, strerror(errno));
		}
	/* repeat read to get full message */
	} while (1);
#if (CFG_GPIO_ENABLE == 1)
	pthread_join(gpio_thread_id, NULL);
#endif
	close(dev_info->rs422_fd);
	return 0;
}
