#include "ratetable.h"

int ratetable_init(struct ratetable_eqmt_info_t *eqmt) {
    eqmt->hwil_ratio_data = 1000;
    eqmt->hwil_type_comm_input = RATETABLE_REALTIME_TYPE_INPUT_RS422;
    eqmt->hwil_tx_speed_baud = 921600;
    eqmt->hwil_send_freq = 100;
    eqmt->movement_mode = RATETABLE_MOVEMENT_MODE_RATE;
    return EXIT_SUCCESS;
}

int ratetable_init_input_file(FILE **stream) {
    char line[1024];
    *stream = fopen("log_rate_3axis.csv", "r");
    if (*stream == NULL) {
        fprintf(stderr, "File not exists !!!\n");
        return EXIT_FAILURE;
    }
    fgets(line, 1024, *stream);
    printf(" %s\n", line);
    return EXIT_SUCCESS;
}

int ratetable_convert_csv_to_motdata(struct ratetable_eqmt_info_t *eqmt, FILE *stream) {
    const char delimiter[2] = ",";
    char *token;
    char line[1024];
    char *saveptr = NULL;
    double axis[3];
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    if (fgets(line, 1024, stream) == NULL) {
        return EXIT_SUCCESS;
    }

    /* xet the first token */
    token = strtok_r(line, delimiter, &saveptr);
    axis[0] = atof(token);
    motion_data->hwil_input[0] =  ROUND_32BIT(axis[0] * eqmt->hwil_ratio_data);

    token = strtok_r(NULL, delimiter, &saveptr);
    axis[1] = atof(token);
    motion_data->hwil_input[1] =  ROUND_32BIT(axis[1] * eqmt->hwil_ratio_data);

    token = strtok_r(NULL, delimiter, &saveptr);
    axis[2] = atof(token);
    motion_data->hwil_input[2] =  ROUND_32BIT(axis[2] * eqmt->hwil_ratio_data);
    //  fprintf(stderr, "[%s:%d] \n", __FUNCTION__, __LINE__,motion_data->hwil_input[0], motion_data->hwil_input[1], motion_data->hwil_input[2]);
    return EXIT_SUCCESS;
}

int ratetable_layer2_frame_direct_transfer(struct icf_ctrlblk_t *C, struct ratetable_eqmt_info_t *eqmt) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    uint32_t send_size = 4;
    uint8_t tx_buffer[4]= {0};
    icf_tx_direct(C, EGSE_RATETBL_X_SW_QIDX, (uint8_t *)&motion_data->hwil_input[0], send_size);
    icf_tx_direct(C, EGSE_RATETBL_Y_SW_QIDX, (uint8_t *)&motion_data->hwil_input[1], send_size);
    icf_tx_direct(C, EGSE_RATETBL_X_SW_QIDX, (uint8_t *)&motion_data->hwil_input[2], send_size);

    return 0;
}

