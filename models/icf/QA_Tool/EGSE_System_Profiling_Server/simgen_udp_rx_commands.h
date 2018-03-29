#ifndef MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_RX_COMMANDS_H_
#define MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_RX_COMMANDS_H_
 
struct Mot_data {
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

struct Motb_data {
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


//-------------------------------
enum SigType {
    gps = 0,
    waas,
    egnos,
    msas,
    glonass,
    galileo,
    gps_2,
    gps_3,
    gps_4,
    gps_5,
    gps_6,
    gps_7,
    gps_8,
    gps_9,
    gps_10,
    gps_11,
    gps_12
};

enum Mode {
    svid = 0,
    channel
};

enum Applicability {
    single = 0,
    all
};

enum Frequency {
    L1 = 0,
    L2,
    L5
};	
//------------------------------- 
struct Mod_data { 
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

//-------------------------------	
typedef enum {
    mot = 0,
    motb,
    mod
} Type;

enum Time_action {
    action_immediately = 0,
    action_at_timestamp
};
//-------------------------------		

struct UDP_Command {
    Type type_;
    enum Time_action     time_action_;
    unsigned int    time_of_validity_ms_;

    // Starts from 1..n
    unsigned int    vehicle_id_;

    union {
    	struct Mot_data    mot_;
    	struct Motb_data   motb_;
    	struct Mod_data    mod_;

    } data;
    int latency_wrt_tov_and_current_tir_ms_;
};

#endif  //  MODELS_EQUIPMENT_PROTOCOL_INCLUDE_SIMGEN_UDP_RX_COMMANDS_H_
