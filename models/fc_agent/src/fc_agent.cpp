#include "fc_agent.hh"

struct fc_agent_can_format_info_t fc_agent_can_mapping_tbl[] = {
    {1, FC_to_TVC_II_NO1, TVC_START, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_LIFTOFF},
    {1, FC_to_PFS_III, PFS_24V_PWR,  {0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_HOT_STAGING},
    {1, FC_to_PFS_II, PFS_BALL_VALVES_ONOFF_REAL, {0xBC, 0x23, 0xAD, 0x14, 0xAA, 0xFF, 0xFF}, FLIGHT_EVENT_CODE_S3_SEPERATION},
    {1, FC_to_ORDNANCE_FAIRING_III, ORDNANCE_FAIRING_NOSECONE_CTRL_OPEN_FAIRING_REAL, {0xDC, 0x43, 0xBA, 0x21, 0xAA, 0xFF, 0xFF}, FLIGHT_EVENT_FAIRING_JETTSION}
};

struct fc_agent_can_format_info_t *fc_agent_find_can_format(uint64_t event_code) {
    struct fc_agent_can_format_info_t *obj = NULL;
    int tbl_size = sizeof(fc_agent_can_mapping_tbl)/sizeof(struct fc_agent_can_format_info_t);
    int idx = 0;
    for (idx = 0; idx < tbl_size; ++idx) {
        obj = &fc_agent_can_mapping_tbl[idx];
        if (obj->event_code == event_code) {
            return obj;
        }
    }
    return NULL;
}

int fc_agent_tvc_cmd_movement(struct can_frame *tvc_no1_frame, struct can_frame *tvc_no2_frame, void *data_base) {
    double theta_a_rad, theta_b_rad, theta_c_rad, theta_d_rad;
    int16_t theta_a_cnt, theta_b_cnt, theta_c_cnt, theta_d_cnt;
    refactor_downlink_packet_t *ctl_tvc_db = static_cast<refactor_downlink_packet_t * >(data_base);
    theta_a_rad = ctl_tvc_db->theta_a_cmd;
    theta_b_rad = ctl_tvc_db->theta_b_cmd;
    theta_c_rad = ctl_tvc_db->theta_c_cmd;
    theta_d_rad = ctl_tvc_db->theta_d_cmd;

    theta_a_cnt = TRUNCAT_16BIT(theta_a_rad * 180 / PI / TVC_DSP_RESOLUTION);
    theta_b_cnt = TRUNCAT_16BIT(theta_b_rad * 180 / PI / TVC_DSP_RESOLUTION);
    theta_c_cnt = TRUNCAT_16BIT(theta_c_rad * 180 / PI / TVC_DSP_RESOLUTION);
    theta_d_cnt = TRUNCAT_16BIT(theta_d_rad * 180 / PI / TVC_DSP_RESOLUTION);

    //  fprintf(stderr, "%d %d %d %d\n", theta_a_cnt, theta_c_cnt, theta_b_cnt, theta_d_cnt);
    //  fprintf(stderr, "%f %f %f %f\n", theta_a_rad, theta_c_rad, theta_b_rad, theta_d_rad);
    /*Frame tvc no1*/
    /* Target ID */
    tvc_no1_frame->can_id  = FC_to_TVC_III_NO1;
    tvc_no1_frame->can_dlc = 8;
    /* Task Command */
    tvc_no1_frame->data[0] = TVC_MOVEMENT_FAKE;
    /* TVC_no1 pitch => theta_a*/
    copy_buffer_htons(&tvc_no1_frame->data[1], reinterpret_cast<uint16_t *>(&theta_a_cnt));
    /* TVC_no1 yaw => theta_b*/
    copy_buffer_htons(&tvc_no1_frame->data[3], reinterpret_cast<uint16_t *>(&theta_b_cnt));

    /*Frame tvc no2*/
    /* Target ID */
    tvc_no2_frame->can_id  = FC_to_TVC_III_NO2;
    tvc_no2_frame->can_dlc = 8;
    /* Task Command */
    tvc_no2_frame->data[0] = TVC_MOVEMENT_FAKE;
    /* TVC_no2 pitch => theta_c*/
    copy_buffer_htons(&tvc_no2_frame->data[1], reinterpret_cast<uint16_t *>(&theta_c_cnt));
    /* TVC_no2 yaw => theta_d*/
    copy_buffer_htons(&tvc_no2_frame->data[3], reinterpret_cast<uint16_t *>(&theta_d_cnt));
    return 0;
}

int fc_agent_flight_event_can_cmd_generate(struct can_frame *event_frame, uint64_t event_code) {
        struct fc_agent_can_format_info_t *obj = NULL;
        /* Event_frame */
        obj = fc_agent_find_can_format(event_code);
        if (obj == NULL)
            return 0;
        if (obj->rest_of_trigger > 0) {
            obj->rest_of_trigger--;
            event_frame->can_id = obj->canid;
            event_frame->can_dlc = 8;
            event_frame->data[0] = obj->taskcmd;
            memcpy(&event_frame->data[1], &obj->content[0], 7);
            return 1;
        }
        return 0;
}
