#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_CMD_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_CMD_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef enum _SIMGEN_UDP_CMD_TYPE_ENUM {
    UDP_CMD_MOT = 0,
    UDP_CMD_MOTB,
    UDP_CMD_MOD
} SIMGEN_UDP_CMD_TYPE_ENUM;

typedef enum _SIMGEN_TIME_ACTION_ENUM {
    action_immediately_enum = 0,
    action_at_timestamp_enum = 1
}SIMGEN_TIME_ACTION_ENUM;

struct mot_data_t {
    double position_ecef_xyz_              [ 3 ];
    double velocity_mps_xyz_               [ 3 ];
    double acceleration_mps2_xyz_          [ 3 ];
    double jerk_mps3_xyz_                  [ 3 ];
    double heb_                            [ 3 ];

    // About body axis
    double angular_velocity_radps_xyz_     [ 3 ];
    double angular_acceleration_radps_xyz_ [ 3 ];
    double angular_jerk_radsps_xyz_        [ 3 ];
};

struct motb_data_t {
    double latitude_;
    double longitude_;
    double height_;

    double velocity_ned_mps_               [ 3 ];
    double acceleration_ned_mps2_          [ 3 ];
    double jerk_ned_mps3_                  [ 3 ];
    double heb_                            [ 3 ];

    // About body axis
    double angular_velocity_radps_xyz_     [ 3 ];
    double angular_acceleration_radps_xyz_ [ 3 ];
    double angular_jerk_radsps_xyz_        [ 3 ];
};

struct mod_data_t { 
    // Starts from 1
    unsigned int    antenna_id_;

    enum SigType         signal_type_;
    unsigned int    svid_or_channel_no_;
    unsigned int    multipath_index_;
    enum Mode            mode_;
    enum Applicability   all_channels_;

    enum Frequency       frequency_;
    enum Applicability   all_frequencies_;

    double          signal_level_;
    double          carrier_offset_;
    double          code_offset_;
};

struct simgen_udp_command_t {
    SIMGEN_UDP_CMD_TYPE_ENUM    type_;
    SIMGEN_TIME_ACTION_ENUM     time_action_;
    uint32_t    time_of_validity_ms_;

    // Starts from 1..n
    uint32_t   vehicle_id_;

    union {
        struct mot_data_t    mot_;
        struct motb_data_t   motb_;
        struct mod_data_t    mod_;
    } motion_data;
    int latency_wrt_tov_and_current_tir_ms_;
};

#ifdef __cplusplus
extern "C" {
#endif

/* Export Function*/

#ifdef __cplusplus
}
#endif


#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_CMD_H_
