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