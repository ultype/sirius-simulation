#include "simgen_remote.h"


char   cmd_mot_t0[1024];
char   cmd_resp[1024];
static const char  *g_remote_motion_cmd_list[3] = {"MOT", "MOTB", "AIDING_OFFSET"};
static const char cmd_RU[] = "RU\n";
static const char cmd_TR[] = "TR, 0\n";
static const char cmd_AR[] = "AR\n";
static const char cmd_SET_TIOP[] = "SET_TIOP,0,2,GATED,1PPS\n";
static const char cmd_EN[] = "-,EN,1\n";

static int simgen_default_remote_data(struct simgen_motion_data_t *motion_info) {
    motion_info->ts.day = 0; 
    motion_info->ts.hour = 0;
    motion_info->ts.minute = 0;
    motion_info->ts.second = 0;
    motion_info->ts.milli = 0;
    motion_info->cmd_idx = REMOTE_MOTION_CMD_MOT;
    motion_info->vehicle_id = 1;
    motion_info->position_xyz[0] =  4288466.6747904532;
    motion_info->position_xyz[1] = -2873655.5571095324; 
    motion_info->position_xyz[2] = -4871760.5248780008;
    motion_info->velocity_xyz[0] = 3148.5166339857; 
    motion_info->velocity_xyz[1] = -4356.3915379860;
    motion_info->velocity_xyz[2] = 5341.2059223289;
    motion_info->acceleration_xyz[0] = -5.3838731211; 
    motion_info->acceleration_xyz[1] = 2.7228053242;
    motion_info->acceleration_xyz[2] = 5.4345273734;
    motion_info->jerk_xyz[0] = 0.0;
    motion_info->jerk_xyz[1] = 0.0;
    motion_info->jerk_xyz[2] = 0.0;
    motion_info->heb[0] = 2.9412651848; 
    motion_info->heb[1] = 0.0027315089;
    motion_info->heb[2] = 3.1411685628;
    motion_info->angular_velocity[0] = 0.0043633231; 
    motion_info->angular_velocity[1] = 0.0043637185;
    motion_info->angular_velocity[2] = 0.0043629277;
    motion_info->angular_acceleration[0] = 0; 
    motion_info->angular_acceleration[1] = 0; 
    motion_info->angular_acceleration[2] = 0;
    motion_info->angular_jerk[0] = 0; 
    motion_info->angular_jerk[1] = 0; 
    motion_info->angular_jerk[2] = 0;
    return 0;

}

static int simgen_remote_motion_cmd_gen(void *data, char *cmdbuff) {
    struct simgen_motion_data_t *motion_info = (struct simgen_motion_data_t *)data;
    char veh_mot_str[32];
    strncpy(veh_mot_str, VEH_MOT(1), 32);
    sprintf(cmdbuff, "%1d %2d:%2d:%2d.%3d,\
        %s,%s,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f\n",
        motion_info->ts.day, motion_info->ts.hour, motion_info->ts.minute, motion_info->ts.second, motion_info->ts.milli,
        g_remote_motion_cmd_list[motion_info->cmd_idx], veh_mot_str,
        motion_info->position_xyz[0], motion_info->position_xyz[1], motion_info->position_xyz[2],
        motion_info->velocity_xyz[0], motion_info->velocity_xyz[1], motion_info->velocity_xyz[2],
        motion_info->acceleration_xyz[0], motion_info->acceleration_xyz[1], motion_info->acceleration_xyz[2],
        motion_info->jerk_xyz[0], motion_info->jerk_xyz[1], motion_info->jerk_xyz[2],
        motion_info->heb[0], motion_info->heb[1], motion_info->heb[2],
        motion_info->angular_velocity[0], motion_info->angular_velocity[1], motion_info->angular_velocity[2],
        motion_info->angular_acceleration[0], motion_info->angular_acceleration[1], motion_info->angular_acceleration[2],
        motion_info->angular_jerk[0], motion_info->angular_jerk[1], motion_info->angular_jerk[2]);
    return 0;
}

static int simgen_udp_motion_cmd_gen(void *data, struct simgen_udp_command_t *udp_cmd) {

    struct simgen_motion_data_t *motion_info = (struct simgen_motion_data_t *)data;
    udp_cmd->type_ = UDP_CMD_MOT;
    udp_cmd->time_action_ = ACTION_AT_TIMESTAMP_ENUM;
    udp_cmd->time_of_validity_ms_ = ((motion_info->ts.hour * 60 + motion_info->ts.minute) * 60) * 1000 +
                                      motion_info->ts.second * 1000 + motion_info->ts.milli;

    udp_cmd->vehicle_id_ = motion_info->vehicle_id;
    udp_cmd->latency_wrt_tov_and_current_tir_ms_ = 0;

    memcpy(&udp_cmd->data.mot_.position_ecef_xyz_, &motion_info->position_xyz, sizeof(motion_info->position_xyz));
    memcpy(&udp_cmd->data.mot_.velocity_mps_xyz_, &motion_info->velocity_xyz, sizeof(motion_info->velocity_xyz));
    memcpy(&udp_cmd->data.mot_.acceleration_mps2_xyz_, &motion_info->acceleration_xyz, sizeof(motion_info->acceleration_xyz));
    memcpy(&udp_cmd->data.mot_.jerk_mps3_xyz_, &motion_info->jerk_xyz, sizeof(motion_info->jerk_xyz));
    memcpy(&udp_cmd->data.mot_.heb_, &motion_info->heb, sizeof(motion_info->heb));
    memcpy(&udp_cmd->data.mot_.angular_velocity_radps_xyz_, &motion_info->angular_velocity, sizeof(motion_info->angular_velocity));
    memcpy(&udp_cmd->data.mot_.angular_acceleration_radps_xyz_, &motion_info->angular_acceleration, sizeof(motion_info->angular_acceleration));
    memcpy(&udp_cmd->data.mot_.angular_jerk_radsps_xyz_, &motion_info->angular_jerk, sizeof(motion_info->angular_jerk));

    return 0;
}

static int simgen_create_remote_cmd_channel(struct simgen_eqmt_info_t *eqmt_info, char *ifname, int net_port) {
    int err;
    int retry_cnt = 0;
    if ((eqmt_info->remote_cmd_channel_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        perror("Error while opening remote_cmd_channel");
        err = eqmt_info->remote_cmd_channel_fd;
        goto REMOTE_FAIL;
    }
    eqmt_info->remote_cmd_addr.sin_family = PF_INET;
    eqmt_info->remote_cmd_addr.sin_port = htons(net_port);
    if (inet_pton(AF_INET, ifname, &eqmt_info->remote_cmd_addr.sin_addr.s_addr) == 0) {
        fprintf(stderr, "Invalid IP adress\n");
        goto REMOTE_FAIL;
    }
    while (retry_cnt < 5) {
        err = connect(eqmt_info->remote_cmd_channel_fd, (struct sockaddr *)&eqmt_info->remote_cmd_addr,
                      sizeof(eqmt_info->remote_cmd_addr));
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
    fprintf(stderr, "create_remote_cmd_channel: Connection %s:%d\n", ifname, net_port);
    return EXIT_SUCCESS;
REMOTE_FAIL:
    fprintf(stderr, "create_remote_cmd_channel: Connection error !!\n");
    return EXIT_FAILURE;
}

static int simgen_create_udp_cmd_channel(struct simgen_eqmt_info_t *eqmt_info, char *ifname, int net_port) {
    int err;
    int retry_cnt = 0;
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


static int remote_cmd_recv(struct simgen_eqmt_info_t *eqmt_info, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    while (offset < buff_size) {
        if ((rdlen = recv(eqmt_info->remote_cmd_channel_fd, rx_buff + offset, buff_size - offset, 0)) < 0) {
            if (errno == EINTR)
                rdlen = 0;
            else
                return -1;
        } else if(rdlen == 0) {
            break;
        }
        offset += rdlen;
    }
    return offset;
}

static int remote_cmd_send(struct simgen_eqmt_info_t *eqmt_info, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    while (offset < frame_len) {
        if ((wdlen = send(eqmt_info->remote_cmd_channel_fd, payload + offset, frame_len - offset, 0)) < 0) {
            if (wdlen < 0 && errno == EINTR)
                wdlen = 0;
            else
                return -1;
        }
        offset += wdlen;
    }
    return offset;
}

static int udp_cmd_sendto(struct simgen_eqmt_info_t *eqmt_info, uint8_t *payload, uint32_t frame_len) {
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

int simgen_remote_wait_cmd_resp(struct simgen_eqmt_info_t *eqmt_info, const char *tx_cmd, char *rx_buff) {
    int ret;
    printf("SimGen cmd send: %s", tx_cmd);
    if ((ret = recv(eqmt_info->remote_cmd_channel_fd, (uint8_t *)rx_buff, 512, 0)) < 0) {
        fprintf(stderr, "[%s:%d] Error cmd response !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    rx_buff[ret] = '\0';
    sleep(1);
    printf("SimGen cmd resp: %s \n\n", rx_buff);
    return 0;
}

int simgen_equipment_init(struct simgen_eqmt_info_t *eqmt_info, void *data) {
    uint8_t *cmd_payload;
    uint32_t cmd_len;

    struct simgen_motion_data_t *motion_info = (struct simgen_motion_data_t * )data;
    
    /* Create TCP Socket */
    fprintf(stderr, "Connect to TCP Server....\n");
    if (simgen_create_remote_cmd_channel(eqmt_info, SIMGEN_IP, SIMGEN_PORT) < 0) {
        fprintf(stderr, "[%s:%d] Fail !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    /* Create UDP socket */
    fprintf(stderr, "Connect to UDP Server....\n");
    if (simgen_create_udp_cmd_channel(eqmt_info, SIMGEN_IP, SIMGEN_PORT) < 0) {
        fprintf(stderr, "[%s:%d] Fail !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }

    /* Send t0 mot by TCP*/
    simgen_default_remote_data(motion_info);
    simgen_remote_motion_cmd_gen(motion_info, cmd_mot_t0);
    cmd_payload = (uint8_t *)cmd_mot_t0;
    cmd_len = strlen(cmd_mot_t0);
    if (remote_cmd_send(eqmt_info, cmd_payload, cmd_len) < cmd_len) {
        fprintf(stderr, "[%s:%d] Error cmd send !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    simgen_remote_wait_cmd_resp(eqmt_info, cmd_mot_t0, cmd_resp);

    /* Send t1 mot */

    /*Send TR command by TCP */
    cmd_payload = (uint8_t *)cmd_TR;
    cmd_len = strlen(cmd_TR);
    if (remote_cmd_send(eqmt_info, cmd_payload, cmd_len) < cmd_len) {
        fprintf(stderr, "[%s:%d] Error cmd send !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }

    simgen_remote_wait_cmd_resp(eqmt_info, cmd_TR, cmd_resp);

    /* Send SET_TIOP commnad by TCP*/
    cmd_payload = (uint8_t *)cmd_SET_TIOP;
    cmd_len = strlen(cmd_SET_TIOP);
    if (remote_cmd_send(eqmt_info, cmd_payload, cmd_len) < cmd_len) {
        fprintf(stderr, "[%s:%d] Error cmd send !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    simgen_remote_wait_cmd_resp(eqmt_info, cmd_SET_TIOP, cmd_resp);

    /* Send AR commnad by TCP*/
    cmd_payload = (uint8_t *)cmd_AR;
    cmd_len = strlen(cmd_AR);
    if (remote_cmd_send(eqmt_info, cmd_payload, cmd_len) < cmd_len) {
        fprintf(stderr, "[%s:%d] Error cmd send !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    simgen_remote_wait_cmd_resp(eqmt_info, cmd_AR, cmd_resp);

    /* Send RU commnad by TCP*/
    cmd_payload = (uint8_t *)cmd_RU;
    cmd_len = strlen(cmd_RU);
    if (remote_cmd_send(eqmt_info, cmd_payload, cmd_len) < cmd_len) {
        fprintf(stderr, "[%s:%d] Error cmd send !!\n", __FUNCTION__, __LINE__);
        return EXIT_FAILURE;
    }
    simgen_remote_wait_cmd_resp(eqmt_info, cmd_RU, cmd_resp);
    return EXIT_SUCCESS;
}

int simgen_motion_data_sendto(struct simgen_eqmt_info_t *eqmt_info, void *data) {
    struct simgen_udp_command_t udp_cmd;
    uint32_t send_size = sizeof(struct simgen_udp_command_t);
    simgen_udp_motion_cmd_gen(data, &udp_cmd);
    if (udp_cmd_sendto(eqmt_info, (uint8_t *)&udp_cmd, sizeof(struct simgen_udp_command_t)) < send_size) {
        fprintf(stderr, "[%s:%d] Motion data send error !!!\n" , __FUNCTION__, __LINE__);
    }
    return EXIT_SUCCESS;
}