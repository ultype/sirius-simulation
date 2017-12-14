#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <time.h>
#include <signal.h>
#include "icf_utility.h"
#include "socket_can.h"
#if __STDC_NO_ATOMICS__ != 1
#include <stdatomic.h>
#endif

#if __STDC_NO_ATOMICS__ == 1
#error "Atomics is not supported"
#elif ATOMIC_INT_LOCK_FREE == 0
#error "int is never lock-free"
#endif


int esps2egse_cmd_init(struct esps2egse_data_t *cmd) {
    uint32_t i;
#if CONFIG_ESPS_HEADER_ENABLE
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
#else
    for (i = 0; i < 8; ++i) {
        cmd->single_cmd[i] = i;
    }
#endif
    return 0;
}

uint32_t ping_pong = 0;
int32_t can_data_send_scatter(int fd, uint8_t *payload, uint32_t data_len) {
    uint32_t sent_len = 0, cur_len = 0, nbytes = 0, i = 0;
    uint32_t sent_len_max = CAN_MAX_DLEN;
    struct can_frame frame;
    int ret;

    frame.can_id  = (ping_pong++ & 0x1)? 0x141:0x142;
    frame.can_dlc = 8;
    while (1) {
        sent_len = (data_len - cur_len) >=  sent_len_max ?
                   sent_len_max : (data_len - cur_len);
        if (sent_len > 0) {
            memset(&frame.data, 0, sent_len_max);
            for (i = 0; i < sent_len; i++, payload++) {
                frame.data[i] = *payload;
            }
#if 1
            frame.data[1] = 0x1f;
#endif
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


#define CLOCKID CLOCK_REALTIME
#define SIG SIGUSR1
#define errExit(msg)    do { perror(msg); exit(EXIT_FAILURE);} while (0)
timer_t timerid;
struct itimerspec its;
struct sigevent sev;
struct sigaction sa;
atomic_int send_flag = ATOMIC_VAR_INIT(0);

static void tx_can_handler(int sig, siginfo_t *si, void *uc) {
    if (si->si_value.sival_ptr != &timerid) {
        return;
    } else {
        /* Critical section need to protect??*/
        send_flag = 1;
        /* Critical section END*/
        if (timer_settime(timerid, 0, &its, NULL) == -1)
            errExit("timer_settime");
    }

}

int main(int argc, char **argv) {
    int nbytes = 0;
    char *ifname = "can1";
    struct esps2egse_data_t esps_cmd;
    struct can_device_info_t can_device;

    uint8_t *p_esps_cmd;
    uint32_t data_len, crc;
    uint8_t *tx_buffer;
    uint32_t buf_offset = 0;
    uint32_t esps2egse_full_size;
    uint32_t tx_loop = 40;
    int opt;
#if ATOMIC_INT_LOCK_FREE == 1
    if (!atomic_is_lock_free(&e_flag)) {
        return EXIT_FAILURE;
    }
#endif
    while ((opt = getopt(argc, argv, "I:l:")) != -1) {
        switch (opt) {
        case 'I':
            ifname = optarg;
            break;
        case 'l':
            tx_loop = atoi(optarg);
            break;
        default: /* '?' */
            printf("Use Default sendCan interface can1, tx_loop = 40\n");
        }
    }
    p_esps_cmd = (uint8_t *)&esps_cmd;
    socket_can_init(&can_device, "can1", 4);

    esps2egse_cmd_init((struct esps2egse_data_t *)p_esps_cmd);
    data_len = sizeof(struct esps2egse_data_t);
    /* start to do crc procedure */
    crc = crc32_create(p_esps_cmd, data_len) ;
    printf("TX CAN CRC without header: 0x%x\n", crc);

    esps2egse_full_size = (CONFIG_ESPS_HEADER_ENABLE ? ESPS2EGSE_HEADER_SIZE : 0) + sizeof(struct esps2egse_data_t);
    tx_buffer = calloc(1, esps2egse_full_size);

    buf_offset = 0;
#if CONFIG_ESPS_HEADER_ENABLE
    memcpy(tx_buffer + buf_offset, &data_len, 4);
    buf_offset += 4;
    memcpy(tx_buffer + buf_offset, &crc, 4);
    buf_offset += 4;
#endif /* CONFIG_ESPS_HEADER_ENABLE */
    memcpy(tx_buffer + buf_offset, p_esps_cmd, sizeof(struct esps2egse_data_t));
    printf("Using %s send esps2egse_data_t size: %d\n", ifname, esps2egse_full_size);

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
    its.it_value.tv_nsec = 50000000;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;

    if (timer_settime(timerid, 0, &its, NULL) == -1)
        errExit("timer_settime");
    while (1) {
        if (send_flag && tx_loop > 0) {
            /* Critical section need to protect??*/
            send_flag = 0;
            FTRACE_TIME_STAMP(510);
            nbytes = can_data_send_scatter(can_device.can_fd, tx_buffer, esps2egse_full_size);
            printf("[%lf:%02d] TX %s [%d/%d] bytes just send. \n", get_curr_time(), tx_loop , ifname , esps2egse_full_size, nbytes);
            /* Critical section END*/
            tx_loop--;
        }
        if (tx_loop == 0) {
            timer_delete(timerid);
            break;
        }
    }
    free(tx_buffer);
    close(can_device.can_fd);
    return 0;
}
