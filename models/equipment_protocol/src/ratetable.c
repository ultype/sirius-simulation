#include "ratetable.h"
#include <math.h>
static double __PI = 3.1415926536;
int ratetable_init(struct ratetable_eqmt_info_t *eqmt) {
    eqmt->hwil_ratio_data = 10000;
    eqmt->hwil_type_comm_input = RATETABLE_REALTIME_TYPE_INPUT_RS422;
    eqmt->hwil_tx_speed_baud = 230400;
    eqmt->hwil_send_freq = 200;
    eqmt->movement_mode = RATETABLE_MOVEMENT_MODE_RATE;
    memset(&eqmt->mot_data, 0, sizeof(struct ratetable_motion_data_t));
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

int ratetable_cmd_step_check(struct ratetable_eqmt_info_t *eqmt, double degree) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;

    motion_data->hwil_input_deg[0] = degree;
    motion_data->hwil_input[0] =  ROUND_32BIT(motion_data->hwil_input_deg[0] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[1] = degree;
    motion_data->hwil_input[1] =  ROUND_32BIT(motion_data->hwil_input_deg[1] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[2] = degree;
    motion_data->hwil_input[2] =  ROUND_32BIT(motion_data->hwil_input_deg[2] * eqmt->hwil_ratio_data);
    //  fprintf(stderr, "[EGSE->RT] %d, %d, %d\n", motion_data->hwil_input[0], motion_data->hwil_input[1], motion_data->hwil_input[2]);
    return EXIT_SUCCESS;
}

int ratetable_cmd_ramp_check(struct ratetable_eqmt_info_t *eqmt, double start, double end, int freqency) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    double slope;
    slope = (end - start)/freqency;
    motion_data->hwil_input_deg[0] += slope;
    motion_data->hwil_input[0] =  ROUND_32BIT(motion_data->hwil_input_deg[0] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[1] += slope;
    motion_data->hwil_input[1] =  ROUND_32BIT(motion_data->hwil_input_deg[1] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[2] += slope;
    motion_data->hwil_input[2] =  ROUND_32BIT(motion_data->hwil_input_deg[2] * eqmt->hwil_ratio_data);
    //  fprintf(stderr, "[EGSE->RT] %d, %d, %d\n", motion_data->hwil_input[0], motion_data->hwil_input[1], motion_data->hwil_input[2]);
    return EXIT_SUCCESS;
}

int ratetable_cmd_sine_check(struct ratetable_eqmt_info_t *eqmt, double Wg, int freq, double int_step) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    double sine_value;
    motion_data->time_stamp += int_step;
    sine_value = (Wg / 2.0) * sin(2 * __PI * freq * motion_data->time_stamp);
    motion_data->hwil_input_deg[0] = sine_value;
    motion_data->hwil_input[0] =  ROUND_32BIT(motion_data->hwil_input_deg[0] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[1] = sine_value;
    motion_data->hwil_input[1] =  ROUND_32BIT(motion_data->hwil_input_deg[1] * eqmt->hwil_ratio_data);

    motion_data->hwil_input_deg[2] = sine_value;
    motion_data->hwil_input[2] =  ROUND_32BIT(motion_data->hwil_input_deg[2] * eqmt->hwil_ratio_data);
    //  fprintf(stderr, "[EGSE->RT] %d, %d, %d\n", motion_data->hwil_input[0], motion_data->hwil_input[1], motion_data->hwil_input[2]);
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

    /* get the first token */
    token = strtok_r(line, delimiter, &saveptr);
    motion_data->hwil_input_deg[0] = atof(token) * (180 / __PI);
    motion_data->hwil_input[0] =  ROUND_32BIT(motion_data->hwil_input_deg[0] * eqmt->hwil_ratio_data);

    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->hwil_input_deg[1] = atof(token) * (180 / __PI);
    motion_data->hwil_input[1] =  ROUND_32BIT(motion_data->hwil_input_deg[1] * eqmt->hwil_ratio_data);

    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->hwil_input_deg[2] = atof(token) * (180 / __PI);
    motion_data->hwil_input[2] =  ROUND_32BIT(motion_data->hwil_input_deg[2] * eqmt->hwil_ratio_data);
    //  fprintf(stderr, "[EGSE->RT] %d, %d, %d\n", motion_data->hwil_input[0], motion_data->hwil_input[1], motion_data->hwil_input[2]);
    return EXIT_SUCCESS;
}

int ratetable_layer2_frame_direct_transfer(struct icf_ctrlblk_t *C, struct ratetable_eqmt_info_t *eqmt) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    uint32_t send_size = 4;
    uint8_t tx_buffer[4]= {0};

    copy_buffer_htonl(&tx_buffer, (uint32_t *)&motion_data->hwil_input[0]);
    icf_tx_direct(C, EGSE_RATETBL_X_SW_QIDX, &tx_buffer, send_size);

    memset(tx_buffer, 0, send_size);
    copy_buffer_htonl(&tx_buffer, (uint32_t *)&motion_data->hwil_input[1]);
    icf_tx_direct(C, EGSE_RATETBL_Y_SW_QIDX, &tx_buffer, send_size);

    memset(tx_buffer, 0, send_size);
    copy_buffer_htonl(&tx_buffer, (uint32_t *)&motion_data->hwil_input[2]);
    icf_tx_direct(C, EGSE_RATETBL_Z_SW_QIDX, &tx_buffer, send_size);

    return 0;
}

int ratetable_layer2_frame_received(struct icf_ctrlblk_t *C, struct ratetable_eqmt_info_t *eqmt) {
    struct ratetable_motion_data_t *motion_data = &eqmt->mot_data;
    uint32_t rx_buff_size = 4;
    uint8_t rx_buffer[4]= {0};

    if (icf_rx_dequeue(C, EGSE_RX_RATETBL_X_SW_QIDX, rx_buffer, rx_buff_size) > 0) {
        copy_buffer_ntohl((uint32_t *)&motion_data->hwil_output[0], rx_buffer);
        motion_data->hwil_output_deg[0] = (double)motion_data->hwil_output[0] / eqmt->hwil_ratio_data;
    }

    memset(rx_buffer, 0, rx_buff_size);
    if (icf_rx_dequeue(C, EGSE_RX_RATETBL_Y_SW_QIDX, rx_buffer, rx_buff_size) > 0) {
        copy_buffer_ntohl((uint32_t *)&motion_data->hwil_output[1], rx_buffer);
        motion_data->hwil_output_deg[1] = (double)motion_data->hwil_output[1] / eqmt->hwil_ratio_data;
    }

    memset(rx_buffer, 0, rx_buff_size);
    if (icf_rx_dequeue(C, EGSE_RX_RATETBL_Z_SW_QIDX, rx_buffer, rx_buff_size) > 0) {
        copy_buffer_ntohl((uint32_t *)&motion_data->hwil_output[2], rx_buffer);
        motion_data->hwil_output_deg[2] = (double)motion_data->hwil_output[2] / eqmt->hwil_ratio_data;
    }
    //  fprintf(stderr, "[RT->EGSE] %d, %d, %d\n", motion_data->hwil_output[0], motion_data->hwil_output[1], motion_data->hwil_output[2]);

    return 0;
}
