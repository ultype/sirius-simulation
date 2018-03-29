#include "simgen_remote.h"


char *nRU = "RU\n";
char *nTR = "TR, 0\n";
char *nAR = "AR\n";
char *nSET_TIOP = "SET_TIOP,0,2,GATED,1PPS\n";
char *nEN = "-,EN,1\n";

/* struct UDP_Commnad is defined in SimGen_UDP_rx_commands.h*/
struct UDP_command udp_mot;

/* init MOT */
char *t0 = "0 00:00:00.0000,mot,v1_m1, 4288466.6747904532, -2873655.5571095324, -4871760.5248780008,\
            3148.5166339857, -4356.3915379860, 5341.2059223289, \
            -5.3838731211, 2.7228053242, 5.4345273734, \
            0.0,0.0,0.0, \
            2.9412651848, 0.0027315089, 3.1411685628,\
            0.0043633231, 0.0043637185, 0.0043629277,\
            0.0,0.0,0.0,\
            0.0,0.0,0.0\n";
char tn[512];
char tcp_ack_buff[512];

struct simgen_eqmt_info_t simgen_eqmt;

static const char  *g_motion_cmd_list[3] = {"MOT", "MOTB", "AIDING_OFFSET"}
static const char nRU[] = "RU\n";
static const char nTR[] = "TR, 0\n";
static const char nAR[] = "AR\n";
static const char nSET_TIOP[] = "SET_TIOP,0,2,GATED,1PPS\n";
static const char nEN[] = "-,EN,1\n";
int simgen_motion_remote_cmd_gen(void *data, char *outbuff) {
    struct simgen_remote_motion_cmd_t *motion_info = data;
    char veh_mot_str[32];
    strncpy(veh_mot_str, VEH_MOT(1), 32);
    sprintf(outbuff, "%1d %2d:%2d:%2d.%3d,\
        %s,%s,\
        %20.10f, %20.10f, %20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f,\
        %20.10f,%20.10f,%20.10f\n",
        motion_info->ts.day, motion_info->ts.hour, motion_info->ts.minute, motion_info->ts.second, motion_info->ts.milli,
        g_motion_cmd_list[motion_info->cmd_idx], veh_mot_str,
        motion_info->position_xyz[0], motion_info->position_xyz[1], motion_info->position_xyz[2],
        motion_info->velocity_xyz[0], motion_info->velocity_xyz[1], motion_info->velocity_xyz[2],
        motion_info->acceleration_xyz[0], motion_info->acceleration_xyz[1], motion_info->acceleration_xyz[2],
        motion_info->jerk_xyz[0], motion_info->jerk_xyz[1], motion_info->jerk_xyz[2],
        motion_info->heading, motion_info->elevation, motion_info->bank,
        motion_info->angular_velocity[0], motion_info->angular_velocity[1], motion_info->angular_velocity[2],
        motion_info->angular_acceleration[0], motion_info->angular_acceleration[1], motion_info->angular_acceleration[2],
        motion_info->angular_jerk[0], motion_info->angular_jerk[1], motion_info->angular_jerk[2]);
    return 0;
}


int simgen_create_remote_cmd_channel(struct simgen_eqmt_info_t *eqmt_info, char *ifname, int net_port) {
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
    if (err < 0) {
        goto REMOTE_FAIL;
    fprintf(stderr, "create_remote_cmd_channel: Connection %s:%d\n", ifname, net_port);
    return EXIT_SUCCESS;
REMOTE_FAIL:
    fprintf(stderr, "create_remote_cmd_channel: Connection error !!\n");
    return EXIT_FAILURE;
}

int simgen_create_udp_cmd_channel(struct simgen_eqmt_info_t *eqmt_info, char *ifname, int net_port) {
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
    if (err < 0) {
        goto REMOTE_FAIL;
    fprintf(stderr, "create_remote_cmd_channel: Connection %s:%d\n", ifname, net_port);
    return EXIT_SUCCESS;
REMOTE_FAIL:
    fprintf(stderr, "create_remote_cmd_channel: Connection error !!\n");
    return EXIT_FAILURE;
}


int remote_cmd_recv(struct simgen_eqmt_info_t *eqmt_info, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    do {
        rdlen = recv(eqmt_info->remote_cmd_channel_fd, rx_buff + offset, buff_size - offset, 0);
        offset += rdlen;
    } while (offset < buff_size && rdlen > 0);
    return offset;
}

int remote_cmd_send(struct simgen_eqmt_info_t *eqmt_info, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    while (offset < frame_len) {
        wdlen = send(eqmt_info->remote_cmd_channel_fd, payload + offset, frame_len - offset, 0);
        if (wdlen < 0) {
            //  fprintf(stderr, "[%s:%d] send error: %d\n", __FUNCTION__, __LINE__, wdlen);
            break;
        }
        offset += wdlen;
    }
    return offset;
}

int udp_cmd_sendto(struct simgen_eqmt_info_t *eqmt_info, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    while (offset < frame_len) {
        wdlen = sendto(eqmt_info->udp_cmd_channel_fd, payload + offset, frame_len - offset,
                       0, (struct sockaddr *)&eqmt_info->udp_cmd_addr, sizeof(eqmt_info->udp_cmd_addr));
        if (wdlen < 0) {
            //  fprintf(stderr, "[%s:%d] send error: %d\n", __FUNCTION__, __LINE__, wdlen);
            break;
        }
        offset += wdlen;
    }
    return offset;
}



int simgen_init(void) {
    int ret;
    char *tcp_cmd
    
    /* Create UDP socket */
    if ((ret = CreateUDP_Channel(UDP_ANY_LOCAL_PORT, &SimGen_udp_channel)) != 0) {
        return 0;
    }

    /* Create TCP Socket */
    if ((ret = ConnectToTCPServer(&SimGen_tcp_channel, SIMGEN_PORT, SIMGEN_IP, NULL, NULL, 5000)) != 0) {
        return 0;
    }

    /* Send t0 mot by TCP*/
    tcp_cmd = t0;
    if (ClientTCPWrite(SimGen_tcp_channel, tcp_cmd, strlen(tcp_cmd), 1000) < 0) {
        return 0;
    }

    if ((ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 1000)) < 0) {
        return 0;
    } else {
        tcp_ack_buff[ret] = '\0';
        printf("SimGen TCP response of %s cmd \n%s \n", tcp_cmd, tcp_ack_buff);
    }

    /* Send t1 mot */

    /*Send TR command by TCP */
    tcp_cmd = nTR;
    if (ClientTCPWrite(SimGen_tcp_channel, tcp_cmd, strlen(tcp_cmd), 1000) < 0) {
        return 0;
    }

    if ((ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 1000)) < 0) {
        return 0;
    } else {
        tcp_ack_buff[ret] = '\0';
        printf("SimGen TCP response of %s cmd \n%s \n", tcp_cmd, tcp_ack_buff);
    }

    /* Send SET_TIOP commnad by TCP*/
    tcp_cmd = nSET_TIOP;
    if (ClientTCPWrite(SimGen_tcp_channel, tcp_cmd, strlen(tcp_cmd), 1000) < 0) {
        return 0;
    }

    if ((ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 1000)) < 0) {
        return 0;
    } else {
        tcp_ack_buff[ret] = '\0';
        printf("SimGen TCP response of %s cmd \n%s \n", tcp_cmd, tcp_ack_buff);
    }

    /* Send AR commnad by TCP*/
    tcp_cmd = nAR;
    if (ClientTCPWrite(SimGen_tcp_channel, tcp_cmd, strlen(tcp_cmd), 1000) < 0) {
        return 0;
    }

    if ((ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 40000)) < 0) {
        return 0;
    } else {
        tcp_ack_buff[ret] = '\0';
        printf("SimGen TCP response of %s cmd \n%s \n", tcp_cmd, tcp_ack_buff);
    }

    /* Send RU commnad by TCP*/
    tcp_cmd = nRU;
    if (ClientTCPWrite(SimGen_tcp_channel, tcp_cmd, strlen(tcp_cmd), 1000) < 0) {
        return 0;
    }

    if ((ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 1000)) < 0) {
        return 0;
    } else {
        tcp_ack_buff[ret] = '\0';
        printf("SimGen TCP response of %s cmd \n%s \n", tcp_cmd, tcp_ack_buff);
    }
    return 1;
}

int simgen_send_mot_tcp(void) {
    int ret;
    sprintf(tn, "0 %2d:%2d:%7.4f,mot,v1_m1,\
            %20.10f, %20.10f, %20.10f,\
            %20.10f,%20.10f,%20.10f,\
            %20.10f,%20.10f,%20.10f,\
            0.0,0.0,0.0,\
            %20.10f,%20.10f,%20.10f,\
            %20.10f,%20.10f,%20.10f,\
            0.0,0.0,0.0,\
            0.0,0.0,0.0\n",
            SimGen_hour, SimGen_minute, Simgen_second,
            SimGen_pos_ecef[0], SimGen_pos_ecef[1], SimGen_pos_ecef[2],
            SimGen_vel_ecef[0], SimGen_vel_ecef[1], SimGen_vel_ecef[2],
            SimGen_acc_ecef[0], SimGen_acc_ecef[1], SimGen_acc_ecef[2],
            SimGen_sc_ned[0], SimGen_sc_ned[1], SimGen_sc_ned[2],
            SimGen_sc_rate[0], SimGen_sc_rate[1], SimGen_sc_rate[2]);
    if(ClientTCPWrite(SimGen_tcp_channel, tn, strlen(tn), 1000) < 0) {
        return 0;
    }
    ret = ClientTCPRead(SimGen_tcp_channel, tcp_ack_buff, sizeof(tcp_ack_buff), 10);
    return ret;
}

int simgen_send_mot_udp(void) {
    int ret;
    udp_mot.type = mot;
    udp_mot.time_action = action_at_timestamp;
    udp_mot.time_of_validity_ms_ = ((SimGen_hour * 60 + SimGen_minute) * 60) * 1000 + Simgen_second * 1000;

    udp_mot.vehicle_id_ = 1;
    udp_mot.latency_wrt_tov_and_current_tir_ms = 0;

    udp_mot.data.mot_.position_ecef_xyz_[0] = SimGen_pos_ecef[0];
    udp_mot.data.mot_.position_ecef_xyz_[1] = SimGen_pos_ecef[1];
    udp_mot.data.mot_.position_ecef_xyz_[2] = SimGen_pos_ecef[2];

    udp_mot.data.mot_.velocity_mps_xyz_[0] = SimGen_vel_ecef[0];
    udp_mot.data.mot_.velocity_mps_xyz_[1] = SimGen_vel_ecef[1];
    udp_mot.data.mot_.velocity_mps_xyz_[2] = SimGen_vel_ecef[2];

    udp_mot.data.mot_.acceleration_mps2_xyz_[0] = SimGen_acc_ecef[0];
    udp_mot.data.mot_.acceleration_mps2_xyz_[1] = SimGen_acc_ecef[1];
    udp_mot.data.mot_.acceleration_mps2_xyz_[2] = SimGen_acc_ecef[2];

    udp_mot.data.mot_.jerk_mps3_xyz_[0] = 0;
    udp_mot.data.mot_.jerk_mps3_xyz_[1] = 0;
    udp_mot.data.mot_.jerk_mps3_xyz_[2] = 0;

    udp_mot.data.mot_.heb_[0] = SimGen_sc_ned[0];
    udp_mot.data.mot_.heb_[1] = SimGen_sc_ned[1];
    udp_mot.data.mot_.heb_[2] = SimGen_sc_ned[2];

    udp_mot.data.mot_.angular_velocity_radps_xyz_[0] = SimGen_sc_rate[0];
    udp_mot.data.mot_.angular_velocity_radps_xyz_[1] = SimGen_sc_rate[1];
    udp_mot.data.mot_.angular_velocity_radps_xyz_[2] = SimGen_sc_rate[2];

    udp_mot.data.mot_.angular_acceleration_radps_xyz_[0] = 0;
    udp_mot.data.mot_.angular_acceleration_radps_xyz_[1] = 0;
    udp_mot.data.mot_.angular_acceleration_radps_xyz_[2] = 0;

    udp_mot.data.mot_.angular_jerk_radps_xyz_[0] = 0;
    udp_mot.data.mot_.angular_jerk_radps_xyz_[1] = 0;
    udp_mot.data.mot_.angular_jerk_radps_xyz_[2] = 0;

    if ((ret = UDPWrite(SimGen_udp_channel, SIMGEN_PORT, SIMGEN_IP, (uint32_t *)&udp_mot.type_, sizeof(udp_mot)))) {
        printf("UDP write fail\n");
        return 0;
    }

    return ret;
}