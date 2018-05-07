#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_REMOTE_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_REMOTE_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      simgen remote
LIBRARY DEPENDENCY:
      (
        (../src/simgen_remote.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include "simgen_udp_cmd.h"
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#define SIMGEN_IP "127.0.0.1"
//#define SIMGEN_IP "192.168.0.8"
#define SIMGEN_PORT 15650
#define VEH_MOT(id) "v"#id"_m1"

typedef enum _REMOTE_MOTION_CMD_ENUM {
    REMOTE_MOTION_CMD_MOT = 0,
    REMOTE_MOTION_CMD_MOTB,
    REMOTE_MOTION_CMD_AIDING_OFFSET,
    REMOTE_MOTION_CMD_MAX_NUM
} REMOTE_MOTION_CMD_ENUM;

struct simgen_timestamp_t {
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    double second;
};

struct simgen_gps_start_time_t {
    /* Date */
    uint32_t year;
    uint8_t month;
    uint8_t day;
    /* time */
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

struct simgen_motion_data_t {
    union {
        struct simgen_timestamp_t ts;
        double second;
    } sim_time;
    uint8_t cmd_idx;
    uint8_t vehicle_id;
    double position_xyz[3];         // uint: metres
    double velocity_xyz[3];         // uint: m/s
    double acceleration_xyz[3];     // uint: m/s^2
    double jerk_xyz[3];             // uint: m/s^3
    double heb[3];                  // heading: uint range +/- pi
                                    // elevation: uint range +/- pi/2
                                    // bank: uint range +/- pi
    double angular_velocity[3];     // uint: rad/s
    double angular_acceleration[3]; // uint: rad/s^2
    double angular_jerk[3];         // uint: rad/s^3
};

struct simgen_eqmt_info_t {
    int remote_cmd_channel_fd;
    int udp_cmd_channel_fd;
    struct sockaddr_in remote_cmd_addr;
    struct sockaddr_in udp_cmd_addr;
    struct simgen_motion_data_t motion_data;
    struct simgen_gps_start_time_t gps_start_time;
    uint8_t udp_motion_enable;
};

#ifdef __cplusplus
extern "C" {
#endif
int simgen_remote_cmd_init(struct simgen_eqmt_info_t *eqmt_info, void *data);
int simgen_udp_motion_cmd_gen(void *data, struct simgen_udp_command_t *udp_cmd);
int simgen_default_remote_data(struct simgen_motion_data_t *motion_info);
int simgen_remote_tn_motion_send(struct simgen_eqmt_info_t *eqmt_info, void *data);
void simgen_remote_end_scenario_now(struct simgen_eqmt_info_t *eqmt_info);
int simgen_init_input_file(FILE **stream);
int simgen_convert_csv_to_mot(struct simgen_motion_data_t *motion_data, FILE *stream, int execution);
int simgen_equipment_udp_channel_init(struct simgen_eqmt_info_t *eqmt_info, char *ifname);
int simgen_motion_tn_udp_send(struct simgen_eqmt_info_t *eqmt_info, void *data);
int simgen_remote_cmd_channel_deinit(struct simgen_eqmt_info_t *eqmt_info);
int simgen_udp_channel_deinit(struct simgen_eqmt_info_t *eqmt_info);
#ifdef __cplusplus
}
#endif
#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_REMOTE_H_
