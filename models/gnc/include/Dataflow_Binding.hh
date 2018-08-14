#ifndef __DATAFLOW_BINDING_HH
#define __DATAFLOW_BINDING_HH

#include "Ins.hh"
#include "GPS.hh"
#include "Control.hh"
#include "DM_FSW_Interface.hh"

#define GPS_LINK_decl() void GPSLinkInData(GPS_FSW &gps, refactor_uplink_packet_t &dm_ins_db, INS &ins) { \
    gps.grab_SBIIC = LINK(ins, get_SBIIC); \
    gps.grab_VBIIC = LINK(ins, get_VBIIC); \
    gps.grab_WBICI = LINK(ins, get_WBICI); \
    gps.grab_SBEEC = LINK(ins, get_SBEEC); \
    gps.grab_VBEEC = LINK(ins, get_VBEEC); \
    gps.grab_TEIC = LINK(ins, get_TEIC); \
    gps.grab_transmit_data  = GRAB_VAR(dm_ins_db.gps_con_transmit_data); \
}

#define INS_LINK_decl() void INSLinkInData(INS &ins, refactor_uplink_packet_t &dm_ins_db, GPS_FSW &gps) { \
    ins.grab_SXH                    = LINK(gps           , get_SXH); \
    ins.grab_VXH                    = LINK(gps           , get_VXH); \
    ins.grab_computed_WBIB          = GRAB_VEC3(dm_ins_db.trick_data.gyro_WBICB); \
    ins.grab_error_of_computed_WBIB = GRAB_VEC3(dm_ins_db.trick_data.gyro_EWBIB); \
    ins.grab_computed_FSPB          = GRAB_VEC3(dm_ins_db.accel_FSPCB); \
    ins.grab_error_of_computed_FSPB = GRAB_VEC3(dm_ins_db.accel_EFSPB); \
    ins.grab_PHI                    = GRAB_VEC3(dm_ins_db.trick_data.sdt_phi); \
    ins.grab_PHI_HIGH               = GRAB_VEC3(dm_ins_db.trick_data.sdt_phi_high); \
    ins.grab_PHI_LOW                = GRAB_VEC3(dm_ins_db.trick_data.sdt_phi_low); \
    ins.grab_DELTA_VEL              = GRAB_VEC3(dm_ins_db.trick_data.sdt_delta_vel); \
    ins.grab_gps_update             = GRAB_VAR(dm_ins_db.gps_con_gps_update); \
}

#define CONTROL_LINK_decl() void ControlLinkInData(Control &control, refactor_ins_to_ctl_t &ins_ctl_db) { \
    control.grab_dvbec         = GRAB_VAR(ins_ctl_db.ins_dvbec); \
    control.grab_thtvdcx       = GRAB_VAR(ins_ctl_db.ins_thtvdcx); \
    control.grab_thtbdcx       = GRAB_VAR(ins_ctl_db.ins_thtbdcx); \
    control.grab_phibdcx       = GRAB_VAR(ins_ctl_db.ins_phibdcx); \
    control.grab_psibdcx       = GRAB_VAR(ins_ctl_db.ins_psibdcx); \
    control.grab_TBDQ          = GRAB_VAR(arma::vec4(ins_ctl_db.ins_TBDQ)); \
    control.grab_TBD           = GRAB_MAT33(ins_ctl_db.ins_TBD); \
    control.grab_alphacx       = GRAB_VAR(ins_ctl_db.ins_alphacx); \
    control.grab_TBICI         = GRAB_MAT33(ins_ctl_db.ins_TBICI); \
    control.grab_TBIC          = GRAB_MAT33(ins_ctl_db.ins_TBIC); \
    control.grab_FSPCB         = GRAB_VAR(arma::vec3(ins_ctl_db.accel_FSPCB)); \
    control.grab_computed_WBIB = GRAB_VAR(arma::vec3(ins_ctl_db.trick_data.gyro_WBICB)); \
    control.grab_altc          = GRAB_VAR(ins_ctl_db.ins_altc); \
}

#define INS_SAVE_decl() void INS_SaveOutData(INS &ins, refactor_uplink_packet_t &dm_ins_db, refactor_ins_to_ctl_t &ins_ctl_db) { \
    ins_ctl_db.ins_dvbec = ins.get_dvbec(); \
    ins_ctl_db.ins_thtvdcx = ins.get_thtvdcx(); \
    STORE_MAT33(ins_ctl_db.ins_TBD, ins.get_TBD()); \
    STORE_MAT33(ins_ctl_db.ins_TBICI, ins.get_TBICI()); \
    STORE_MAT33(ins_ctl_db.ins_TBIC, ins.get_TBIC()); \
    STORE_VEC(ins_ctl_db.ins_TBDQ, ins.get_TBDQ()); \
    ins_ctl_db.ins_alphacx = ins.get_alphacx(); \
    ins_ctl_db.ins_thtbdcx = ins.get_thtbdcx(); \
    ins_ctl_db.ins_phibdcx = ins.get_phibdcx(); \
    ins_ctl_db.ins_psibdcx = ins.get_psibdcx(); \
    ins_ctl_db.ins_altc    = ins.get_altc(); \
    memcpy(&ins_ctl_db.accel_FSPCB, \
           &ins_ctl_db.accel_FSPCB, \
           3 * sizeof(double)); \
    ins_ctl_db.trick_data = \
        dm_ins_db.trick_data; \
}

#define CONTROL_SAVE_decl() void Control_SaveOutData(Control &control, refactor_downlink_packet_t &ctl_tvc_db) { \
    ctl_tvc_db.theta_a_cmd = control.get_theta_a_cmd(); \
    ctl_tvc_db.theta_b_cmd = control.get_theta_b_cmd(); \
    ctl_tvc_db.theta_c_cmd = control.get_theta_c_cmd(); \
    ctl_tvc_db.theta_d_cmd = control.get_theta_d_cmd(); \
}

#define DM_SAVE_decl() void DM_SaveOutData(refactor_uplink_packet_t &dm_ins_db) { \
    STORE_VEC(dm_ins_db.accel_FSPCB, accelerometer->get_computed_FSPB()); \
    STORE_VEC(dm_ins_db.accel_EFSPB, accelerometer->get_error_of_computed_FSPB()); \
    dm_ins_db.gps_con_gps_update = gps_con.get_gps_update(); \
    transmit_channel *trans_chan = gps_con.get_transmit_data(); \
    for (int i = 0; i < MAX_CHAN; ++i) { \
        dm_ins_db.gps_con_transmit_data[i] = trans_chan[i]; \
    } \
    STORE_VEC(dm_ins_db.trick_data.gyro_WBICB, gyro->get_computed_WBIB()); \
    STORE_VEC(dm_ins_db.trick_data.gyro_EWBIB, gyro->get_error_of_computed_WBIB()); \
    STORE_VEC(dm_ins_db.trick_data.sdt_phi, sdt->get_PHI()); \
    STORE_VEC(dm_ins_db.trick_data.sdt_phi_high, sdt->get_PHI_HIGH()); \
    STORE_VEC(dm_ins_db.trick_data.sdt_phi_low, sdt->get_PHI_LOW()); \
    STORE_VEC(dm_ins_db.trick_data.sdt_delta_vel, sdt->get_DELTA_VEL()); \
}
#endif
