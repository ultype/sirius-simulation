#ifndef __DM_FSW_INTERFACE_HH__
#define __DM_FSW_INTERFACE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Interface between DM and FSW)
LIBRARY DEPENDENCY:
      (())
PROGRAMMERS:
      (((Ming-Chia Chung) () () () ))
*******************************************************************************/
#include "global_constants.hh"
#include "Transmit_channel.hh"

#if 0  /* TODO: DM is not support new packet format yet*/
typedef struct __attribute__((packed)) {
    double accel_FSPCB[3];
    double gyro_WBICB[3];
    double gyro_EWBIB[3];
    double accel_EFSPB[3];
    uint32_t gps_con_gps_update;
    double sdt_phi[3];
    double sdt_delta_vel[3];
    double sdt_phi_high[3];
    double sdt_phi_low[3];
    transmit_channel gps_con_transmit_data;
} uplink_packet_format_t;
#endif

typedef struct __attribute__((packed)) {
    /* gyro */
    double gyro_WBICB[3];
    double gyro_EWBIB[3];

    /* SDT */
    double sdt_phi[3];
    double sdt_delta_vel[3];
    double sdt_phi_high[3];
    double sdt_phi_low[3];
} refactor_trick_dirty_data_t;

typedef struct __attribute__((packed)) {
    /* accel */
    double accel_FSPCB[3];
    double accel_EFSPB[3];

    /* GPS Constellation */
    uint32_t gps_con_gps_update;
    transmit_channel gps_con_transmit_data[MAX_CHAN];

    refactor_trick_dirty_data_t trick_data;
} refactor_uplink_packet_t;


typedef struct __attribute__((packed)) {
    int flag_ins_clear_gps;

    double ins_dvbec;
    double ins_thtvdcx;
    double ins_thtbdcx;
    double ins_TBDQ[4];
    double ins_TBD[3][3];
    double ins_TBICI[3][3];
    double ins_TBIC[3][3];

    double ins_alphacx;
    double ins_psibdcx;
    double ins_phibdcx;
    double ins_altc;

    double accel_FSPCB[3];
    refactor_trick_dirty_data_t trick_data;
} refactor_ins_to_ctl_t;

typedef struct __attribute__((packed)) {
    double theta_a_cmd;
    double theta_b_cmd;
    double theta_c_cmd;
    double theta_d_cmd;
    uint64_t flight_event_code;
} refactor_downlink_packet_t;

typedef struct __attribute__((packed)) {
    double ctrl_delrcx;
    double ctrl_delecx;
} ctler_to_tvc_t;

typedef struct __attribute__((packed)) {
    double ctrl_delrcx;
    double ctrl_delecx;
    double guid_isEnabled;
    double guid_rcs_mode;
    double guid_e_roll;
    double guid_e_pitch;
    double guid_e_yaw;
} ctler_to_rcs_t;

typedef struct __attribute__((packed)) {
    double temp;
} to_can_t;

typedef struct __attribute__((packed)) {
    double tmp;
} ctler_to_ordnctrl_t;

typedef struct __attribute__((packed)) {
    double temp;
} ctler_to_flowctrl_t;

#endif  // __DM_FSW_INTERFACE_HH__
