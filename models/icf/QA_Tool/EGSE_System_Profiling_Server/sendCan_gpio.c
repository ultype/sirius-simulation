#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <signal.h>
#include "gpio_sync_timer/DIInterrupt.h"

#define BILLION 			1000000000L
#define INLINE                        	__attribute__((always_inline))
#define CAN_BUFFER_SIZE 8
#define FTRACE_TIME_STAMP(id) do { syscall(id);} while (0)
#define CFG_GPIO_ENABLE 1

void clock_get_hw_time(struct timespec *ts) {
    clock_gettime(CLOCK_MONOTONIC, ts);
}

static struct timespec ts;
double get_curr_time(void) {
    clock_get_hw_time(&ts);
    return ts.tv_sec + (double)ts.tv_nsec / (double)BILLION;
}

#define TVC_SIZE 6
#define RCS_SIZE 4
#define II_VALUE_CONTROL_SIZE 3
#define ORDANCE_SIZE 3
/*
*
* EGSE->DM 20pps, RX cmd format from devices. (egse data verify server)
*
*/
struct egse2dm {
    uint8_t III_TVC_1[TVC_SIZE];
    uint8_t III_TVC_2[TVC_SIZE];
    uint8_t III_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t III_valve_control_2[II_VALUE_CONTROL_SIZE];
    uint8_t RCS[RCS_SIZE];
    uint8_t ordance_faring[ORDANCE_SIZE];
    uint8_t ordance_separation[ORDANCE_SIZE];
    uint8_t II_TVC_1[TVC_SIZE];
    uint8_t II_TVC_2[TVC_SIZE];
    uint8_t II_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t II_valve_control_2[II_VALUE_CONTROL_SIZE];
} __attribute__((packed));

struct egse2dm_header_t {
    uint32_t payload_len;
    uint32_t crc;
} __attribute__((packed));

int egse2dm_cmd_init(struct egse2dm *cmd) {
    uint32_t i;
    for (i = 0; i < II_VALUE_CONTROL_SIZE; ++i) {
        cmd->III_valve_control_1[i] = 0x33U;
        cmd->III_valve_control_2[i] = 0x44U;
        cmd->II_valve_control_1[i] = 0xaaU;
        cmd->II_valve_control_2[i] = 0xbbU;
    }

    for (i = 0; i < TVC_SIZE; ++i) {
        cmd->III_TVC_1[i] = 0x11U;
        cmd->III_TVC_2[i] = 0x22U;
        cmd->II_TVC_1[i] = 0x88U;
        cmd->II_TVC_2[i] = 0x99U;
    }

    for (i = 0; i < RCS_SIZE; ++i) {
        cmd->RCS[i] = 0x55U;
    }

    for (i = 0; i < ORDANCE_SIZE; ++i) {
        cmd->ordance_faring[i] = 0x66U;
        cmd->ordance_separation[i] = 0x77U;
    }
    return 0;
}

int32_t can_data_send_scatter(int fd, uint8_t *payload, uint32_t data_len) {
    uint32_t sent_len = 0, cur_len = 0, nbytes = 0, i = 0;
    uint32_t sent_len_max = CAN_BUFFER_SIZE;
    struct can_frame frame;
    int ret;

    frame.can_id  = 0x123;
    frame.can_dlc = 8;
    while (1) {
        sent_len = (data_len - cur_len) >=  sent_len_max ?
                   sent_len_max : (data_len - cur_len);
        if (sent_len > 0) {
            memset(&frame.data, 0, sent_len_max);
            for (i = 0; i < sent_len; i++, payload++) {
                frame.data[i] = *payload;
            }
            ret = write(fd, &frame, sizeof(struct can_frame));
            if (ret < 0)
                goto error;
            cur_len += sent_len;
            nbytes += ret;
        } else {
            ret = nbytes;
            break;
        }
    }
error:
    return ret;
}

uint32_t crc32(uint32_t crc, const char *buf, uint32_t len) {
    uint32_t	crc_30_00;
    uint32_t	crc_31;
    uint32_t	buf_value;

    if (buf == 0) return 0L;

    buf_value =  *((uint32_t *) buf);
    crc_30_00 = ((crc >> 1) ^ (buf_value & 0x7fffffff));
    crc_31	  = (crc & 0x1) ^
                ((crc & (0x1 << 4)) >> 4) ^
                ((crc & (0x1 << 8)) >> 8) ^
                ((crc & (0x1 << 12)) >> 12) ^
                ((crc & (0x1 << 17)) >> 17) ^
                ((crc & (0x1 << 20)) >> 20) ^
                ((crc & (0x1 << 23)) >> 23) ^
                ((crc & (0x1 << 28)) >> 28) ^
                ((buf_value & (0x1 << 31)) >> 31);

    return	( ( (crc_31 << 31) & 0x80000000 ) | (crc_30_00 & 0x7fffffff ));
}

#define CLOCKID CLOCK_REALTIME
#define SIG SIGUSR1
#define errExit(msg)    do { perror(msg); exit(EXIT_FAILURE);} while (0)
timer_t timerid;
struct itimerspec its;
volatile uint32_t send_flag = 0;

static void tx_can_handler(int sig, siginfo_t *si, void *uc) {
    /* Note: calling printf() from a signal handler is not safe
    (and should not be done in production programs), since
    printf() is not async-signal-safe; see signal-safety(7).
    Nevertheless, we use printf() here as a simple way of
    showing that the handler was called. */
#if 0
    int or;
    timer_t *tidp;
    tidp = si->si_value.sival_ptr;
    or = timer_getoverrun(*tidp);
    if ( or == -1) {
        errExit("timer_getoverrun");
    } else {
        printf("[%lf] sig: %d overrun count = %d\n", get_curr_time() , sig , or );
    }
#endif
    if (si->si_value.sival_ptr != &timerid) {
        return;
    } else {
        /* Critical section need to protect??*/
        __sync_bool_compare_and_swap(&send_flag, 0 , 1);
        /* Critical section END*/
#if 0
        if (timer_settime(timerid, 0, &its, NULL) == -1)
            errExit("timer_settime");
#endif
    }

}


#if (CFG_GPIO_ENABLE == 1)
void *gpio_ttl_thread(void *arg) {
    int *ret = NULL;
    gpio_ttl_init_wrapper();
    return ret;
}
#endif

int main(int argc, char **argv) {
    int can_socket;
    int nbytes = 0;
    struct sockaddr_can addr;
    struct ifreq ifr;
    char *ifname = "can1";
    struct egse2dm egse_cmd;
    uint8_t *p_egse_cmd;
    uint32_t data_len, crc;
    uint8_t *tx_buffer;
    uint32_t buf_offset = 0;
    uint32_t egse2dm_full_size;
    uint32_t tx_pps_user_input = 20;
    uint32_t tx_pps = tx_pps_user_input;
    uint32_t system_cycle = 0;
    int opt;
    struct sigevent sev;
    struct sigaction sa;
    struct itimerspec zerotimer = {0};
    int ret;
    while ((opt = getopt(argc, argv, "I:p:c:")) != -1) {
        switch (opt) {
        case 'I':
            ifname = optarg;
            break;
        case 'p':
            tx_pps_user_input = atoi(optarg);
            tx_pps = tx_pps_user_input;
            break;
        case 'c':
            system_cycle = atoi(optarg);
            break;
        default: /* '?' */
            printf("Use Default sendCan interface can1, tx_pps = 20, system_cycle = 10\n");
        }
    }
    if (system_cycle < tx_pps) {
        printf("System_cycle (-c) must larger than tx_pps (-p %d) !!!\n", tx_pps);
        exit(EXIT_FAILURE);
    }
    printf("System_cycle (-c %d), tx_pps (-p %d) !!!\n", system_cycle, tx_pps);

    p_egse_cmd = (uint8_t *)&egse_cmd;
    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    egse2dm_cmd_init((struct egse2dm *)p_egse_cmd);
    data_len = sizeof(struct egse2dm);
    /* start to do crc procedure */
    crc = nbytes = 0;
    while (nbytes < data_len) {
        crc = crc32(crc, (p_egse_cmd + nbytes), 4);
        nbytes += 4;
    }
    printf("TX CAN CRC without header: 0x%x\n", crc);

    egse2dm_full_size = sizeof(struct egse2dm_header_t) + sizeof(struct egse2dm);
    tx_buffer = calloc(1, egse2dm_full_size);

    buf_offset = 0;
    memcpy(tx_buffer + buf_offset, &data_len, 4);
    buf_offset += 4;
    memcpy(tx_buffer + buf_offset, &crc, 4);
    buf_offset += 4;
    memcpy(tx_buffer + buf_offset, p_egse_cmd, sizeof(struct egse2dm));
    printf("Using %s send egse2dm with header size: %d\n", ifname, egse2dm_full_size);

    /* Establish handler for timer signal */
    printf("Establishing handler for signal %d\n", SIG);
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = tx_can_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIG, &sa, NULL) == -1)
        errExit("sigaction");

    /* Create the timer */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIG;
    sev.sigev_value.sival_ptr = &timerid;
    timer_create(CLOCKID, &sev, &timerid);

    printf("timer ID is 0x%lx\n", (long) timerid);
    /* Start the timer */
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = BILLION / tx_pps_user_input;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = its.it_value.tv_nsec ;
#if (CFG_GPIO_ENABLE == 1)
    pthread_t gpio_thread_id;
    ret = pthread_create(&gpio_thread_id, NULL, gpio_ttl_thread, NULL);
    if (ret) {
        printf("ERROR; return code from gpio_thread_id is %d\n", ret);
        exit(-1);
    }
#endif
    while (1) {
        if (__sync_fetch_and_and(&send_flag, 1) && tx_pps > 0) {
            FTRACE_TIME_STAMP(510);
            nbytes = can_data_send_scatter(can_socket, tx_buffer, egse2dm_full_size);
            printf("[%lf:%02d] TX %s %d bytes send. \n", get_curr_time(), tx_pps , ifname , nbytes);
            /* Critical section need to protect??*/
            __sync_bool_compare_and_swap(&send_flag, 1, 0);
            /* Critical section END*/
            tx_pps--;
        }
        if (tx_pps == 0) {
            timer_settime(timerid, 0, &zerotimer, NULL);
            tx_pps = tx_pps_user_input;
            if (system_cycle > tx_pps_user_input)
                system_cycle -= tx_pps_user_input;
            else {
                timer_delete(timerid);
                break;
            }
        }
    }

#if (CFG_GPIO_ENABLE == 1)
    pthread_join(gpio_thread_id, NULL);
#endif
    free(tx_buffer);
    timer_delete(timerid);
    printf("Test Finish Exit the programs !!!!\n");
    return 0;
}
