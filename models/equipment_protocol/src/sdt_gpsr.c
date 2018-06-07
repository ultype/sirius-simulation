#include "sdt_gpsr.h"
#include "gpsr_s_nav_tlm.h"
#include <string.h>

int sdt_gpsr_convert_motion_data_to_tlm(void *data , struct gpsr_s_nav_tlm_frame_t *tlm_frame) {
    struct sdt_gpsr_motion_data_t *mot_data = (struct sdt_gpsr_motion_data_t *)data;

    memset(tlm_frame, 0, sizeof(struct gpsr_s_nav_tlm_frame_t));
    tlm_frame->gps_week_num = mot_data->gps_week_num;
    tlm_frame->gps_time = mot_data->gps_time;
    tlm_frame->validity = mot_data->validity;
    tlm_frame->posx = mot_data->posx;
    tlm_frame->posy = mot_data->posy;
    tlm_frame->posz = mot_data->posz;
    tlm_frame->velx = mot_data->velx;
    tlm_frame->vely = mot_data->vely;
    tlm_frame->velz = mot_data->velz;
    tlm_frame->visibility_satellites_map = mot_data->visibility_satellites_map;
    return 0;
}

static uint8_t sdt_gpsr_checksum(uint8_t *frame, uint32_t size) {
    uint8_t checksum;
    int idx;
    for (checksum = 0, idx = 0; idx < size; idx++) {
        checksum += frame[idx];
    }
    return -checksum;
}

static uint32_t sdt_gpsr_append_nspo_frame_header(void *data, uint8_t *tx_buffer) {
    struct gpsr_s_nav_tlm_frame_t *tlm_frame = (struct gpsr_s_nav_tlm_frame_t *)data;
    struct nspo_gpsr_frame_header_t header;
    struct nspo_gpsr_frame_tail_t   tail;
    uint32_t offset = 0;

    header.start_of_frame[0] = 'N';
    header.start_of_frame[1] = 'S';
    header.start_of_frame[2] = 'P';
    header.start_of_frame[3] = 'O';

    header.attribute_.data_length = sizeof(struct gpsr_s_nav_tlm_frame_t);
    header.attribute_.frame_id = 0;
    header.attribute_.frame_type = NSPO_FRAME_TYPE_TLM;
    memcpy(tx_buffer, &header, sizeof(struct nspo_gpsr_frame_header_t));
    offset += sizeof(struct nspo_gpsr_frame_header_t);

    memcpy(tx_buffer + offset, tlm_frame, sizeof(struct gpsr_s_nav_tlm_frame_t));
    offset += sizeof(struct gpsr_s_nav_tlm_frame_t);

    tail.checksum = sdt_gpsr_checksum(tx_buffer, sizeof(struct gpsr_s_nav_tlm_frame_t) + sizeof(struct nspo_gpsr_frame_header_t));
    memcpy(tx_buffer + offset, &tail, sizeof(struct nspo_gpsr_frame_tail_t));
    offset += sizeof(struct nspo_gpsr_frame_tail_t);
    return offset;
}

int sdt_gpsr_layer2_tlm_frame_transfer(struct icf_ctrlblk_t *C) {
    struct gpsr_s_nav_tlm_frame_t tlm_frame;
    uint32_t send_size;
    uint8_t *tx_buffer = NULL;
    icf_tx_dequeue(C, EGSE_GPSR01_SW_QIDX, &tlm_frame);
    tx_buffer = malloc(sizeof(struct nspo_gpsr_frame_t));
    send_size = sdt_gpsr_append_nspo_frame_header(&tlm_frame, tx_buffer);
    icf_tx_direct(C, EGSE_GPSR01_SW_QIDX, tx_buffer, send_size);
    //  icf_tx_direct(C, EGSE_GPSR02_SW_QIDX, , send_size);
    //  hex_dump("TX TLM Frame", (uint8_t *) , sizeof(struct gpsr_s_nav_tlm_frame_t));
    if (tx_buffer) {
        free(tx_buffer);
    }
    return 0;
}

int sdt_gpsr_init_input_file(FILE **stream) {
    char line[1024];
    *stream = fopen("gpsr_s_nav_tlm.csv", "r");
    if (*stream == NULL) {
        fprintf(stderr, "File not exists !!!\n");
        return EXIT_FAILURE;
    }
    fgets(line, 1024, *stream);
    printf(" %s\n", line);
    return EXIT_SUCCESS;
}

int sdt_gpsr_convert_csv_to_motdata(struct sdt_gpsr_motion_data_t *motion_data, FILE *stream, int execution) {
    const char delimiter[2] = ",";
    char *token;
    char line[1024];
    int idx = 0;
    char *saveptr = NULL;
    if (fgets(line, 1024, stream) == NULL) {
        return EXIT_SUCCESS;
    }

    if (execution > 1) {
        return EXIT_SUCCESS;
    }
    /* get the first token */
    token = strtok_r(line, delimiter, &saveptr);
    motion_data->gps_time = atof(token);
    /* Position */
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->posx = atof(token);
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->posy = atof(token);
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->posz = atof(token);
    /* velocity */
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->velx = atof(token);
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->vely = atof(token);
    token = strtok_r(NULL, delimiter, &saveptr);
    motion_data->velz = atof(token);
    return EXIT_SUCCESS;
}
