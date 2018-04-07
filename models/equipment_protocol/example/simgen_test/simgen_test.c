#include "simgen_remote.h"
#include <unistd.h>
struct simgen_motion_data_t user_data;
struct simgen_eqmt_info_t simgen_eqmt_test;

static int simgen_test_udp_cmd_sendto(struct simgen_eqmt_info_t *eqmt_info, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    while (offset < frame_len) {
        if ((wdlen = sendto(eqmt_info->udp_cmd_channel_fd, payload + offset, frame_len - offset,
             0, (struct sockaddr *)&eqmt_info->udp_cmd_addr, sizeof(eqmt_info->udp_cmd_addr))) < 0) {
            if (wdlen < 0 && errno == EINTR)
                wdlen = 0;
            else
                return -1;
        }
        offset += wdlen;
    }
    return offset;
}

int simgen_test_motion_data_sendto(struct simgen_eqmt_info_t *eqmt_info, void *data) {
    struct simgen_udp_command_t udp_cmd;
    uint32_t send_size = sizeof(struct simgen_udp_command_t);
    simgen_udp_motion_cmd_gen(data, &udp_cmd);
    if (simgen_test_udp_cmd_sendto(eqmt_info, (uint8_t *)&udp_cmd, sizeof(struct simgen_udp_command_t)) < send_size) {
        fprintf(stderr, "[%s:%d] Motion data send error !!!\n" , __FUNCTION__, __LINE__);
    }
    return EXIT_SUCCESS;
}

int simgen_test_equipment_udp_channel_init(struct simgen_eqmt_info_t *eqmt_info, char *ifname, int net_port) {
    /* Create UDP socket */
    int err;
    int retry_cnt = 0;
    fprintf(stderr, "Connect to simgen UDP Server....\n");
    if ((eqmt_info->udp_cmd_channel_fd = socket(AF_INET , SOCK_DGRAM , 0)) < 0) {
        perror("Error while opening remote_cmd_channel");
        err = eqmt_info->udp_cmd_channel_fd;
        goto REMOTE_FAIL;
    }
    eqmt_info->udp_cmd_addr.sin_family = AF_INET;
    eqmt_info->udp_cmd_addr.sin_port = htons(net_port);
    if (inet_pton(AF_INET, ifname, &eqmt_info->udp_cmd_addr.sin_addr.s_addr) == 0) {
        fprintf(stderr, "Invalid IP adress\n");
        goto REMOTE_FAIL;
    }
    while (retry_cnt < 5) {
        err = connect(eqmt_info->udp_cmd_channel_fd, (struct sockaddr *)&eqmt_info->udp_cmd_addr,
                      sizeof(eqmt_info->udp_cmd_addr));
        if (err == 0)
            break;
        if (err < 0) {
            sleep(3);
            retry_cnt++;
            fprintf(stderr, "create_remote_cmd_channel: Connection error retry...%d\n", retry_cnt);
        }
    }
    if (err < 0)
        goto REMOTE_FAIL;
    fprintf(stderr, "simgen_create_udp_cmd_channel: Connection %s:%d\n", ifname, net_port);
    return EXIT_SUCCESS;
REMOTE_FAIL:
    fprintf(stderr, "simgen_create_udp_cmd_channel: Connection error !!\n");
    return EXIT_FAILURE;
}

int main(int argc, char const *argv[]) {
    simgen_default_remote_data(&user_data);
    simgen_test_equipment_udp_channel_init(&simgen_eqmt_test, SIMGEN_IP, SIMGEN_PORT);
    simgen_remote_cmd_init(&simgen_eqmt_test, &user_data);
    while (1) {
        sleep(1);
        simgen_test_motion_data_sendto(&simgen_eqmt_test, &user_data);
    }
    return 0;
}

