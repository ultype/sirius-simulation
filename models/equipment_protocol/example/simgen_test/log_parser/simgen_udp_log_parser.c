#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "simgen_udp_cmd.h"
#include <unistd.h>
#include <math.h>
#define LOG_PARSER_SIMGEN_LATENCY 1
#define LOG_PARSER_GPS_RECEVIER 2
#define LOG_PARSER_DM_NSPO_REFINE 3
const char* getcolumn(char* line, int col) {
    const char* tok;
    for (tok = strtok(line, ",");
            tok && *tok;
            tok = strtok(NULL, ",\n"))
    {
        if (!--col)
            return tok;
    }
    return NULL;
}

int gps_recevier_log_parser(FILE *input_binary) {
    FILE *output_text;
    char *token;
    char line[1024];
    int iter = 0;
    const char delimiter = ',';
    if ((output_text = fopen("gps_recevier.csv", "w")) == NULL) {
        fclose(input_binary);
        return EXIT_FAILURE;
    }
    fprintf(output_text,"GPS_SOW_TIME (sec), SBEE_X, SBEE_Y, SBEE_Z, VBEE_X, VBEE_Y, VBEE_Z, filed_#8, filed_#9, filed_#10\n");
    while(fgets(line, 1024, input_binary) != NULL) {
        if(strstr(line,"$POS,") == NULL)
            continue;
        if (line[strlen(line) - 1] == '\n' ||  line[strlen(line) - 2] == '\r') {
            line[strlen(line) - 2] = line[strlen(line) - 1] = '\0';
        }
        token = strchr(line, delimiter);
        if (iter % 2 == 0)
            fprintf(output_text,"%s", token + 1);
        iter++;
    }
    fclose(output_text);
    return EXIT_SUCCESS;
}

int simgen_udp_latency_log_parser(FILE *input_binary) {
    FILE *output_text;
    int iter;
    struct simgen_udp_command_t udp_cmd;
    if ((output_text = fopen("UDP_mot.csv", "w")) == NULL) {
        fclose(input_binary);
        return EXIT_FAILURE;
    }
    for(iter = 0 ; ; iter++) {
        if (sizeof(udp_cmd) != fread(&udp_cmd, 1, sizeof(udp_cmd), input_binary))
            break ;

        if (iter == 0) {
            fprintf(output_text,"Iteration, sim_time_ms, latency\n");
            continue;
        }

        fprintf(output_text,"%4d, %6d, %d\n", iter - 1, udp_cmd.time_of_validity_ms_,
                udp_cmd.latency_wrt_tov_and_current_tir_ms_);
    }
    fclose(output_text);
    return EXIT_SUCCESS;
}

int rocket_dynamic_nspo_log_parser(FILE *input_binary, double target_value) {
    FILE *output_text;
    char line[1024];
    char field_content[64];
    double start_gps_time;
    int iter = 0;
    if ((output_text = fopen("refine_nspo_log.csv", "w")) == NULL) {
        fclose(input_binary);
        return EXIT_FAILURE;
    }
    fprintf(output_text,"sim time (sec),GPS_SOW_TIME (sec), SBEE_X, SBEE_Y, SBEE_Z, VBEE_X, VBEE_Y, VBEE_Z\n");
    while(fgets(line, 1024, input_binary) != NULL) {
        char* tmp = strdup(line);
        strcpy(field_content, getcolumn(tmp, 2));
        free(tmp);
        start_gps_time = atof(field_content);
        if (fabs(start_gps_time - target_value) < 1e-4) {
            printf("[%s:%d] %s\n", __FUNCTION__, __LINE__, line);
            fprintf(output_text,"%s", line);
            break;
        }
    }
    iter = 1;
    while(fgets(line, 1024, input_binary) != NULL) {
        if (iter % 100 == 0) {
            fprintf(output_text,"%s", line);
        }
        iter++;
    }

    fclose(output_text);
    return EXIT_SUCCESS;
}

int main(int argc, char  * const argv[]) {
    /* parse UDP binary log file */
    FILE *input_binary = NULL;
    char file_name[1024] = {};
    char ch;
    int parsing_mode = 0;
    double target_value;
    while ((ch = getopt(argc, argv, "f:lgn:")) != EOF) {
        switch (ch) {
            case 'f':
                if (optarg == NULL) {
                    fprintf(stderr, "[%s:%d] Need the file argument.\n", __FUNCTION__, __LINE__);
                    goto EXIT;
                }
                printf("-f %s\r\n", optarg);
                strcpy(file_name, optarg);
                if ((input_binary = fopen(file_name, "rb")) == NULL) {
                    fprintf(stderr, "[ERROR] The file name does not exist\n");
                    goto EXIT;
                }
                break;
            case 'l':
                printf("-l simgen latency parser\n");
                parsing_mode = LOG_PARSER_SIMGEN_LATENCY;
                break;
            case 'g':
                printf("-g gps recevier parser\n");
                parsing_mode = LOG_PARSER_GPS_RECEVIER;
                break;
            case 'n':
                printf("-n NSPO DM Log refine\n");
                if (optarg == NULL) {
                    fprintf(stderr, "[%s:%d] Need the target value.\n", __FUNCTION__, __LINE__);
                    goto EXIT;
                }
                parsing_mode = LOG_PARSER_DM_NSPO_REFINE;
                target_value = atof(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option: '%s'\n", optarg);
                goto EXIT;
        }

    }
    switch (parsing_mode) {
        case LOG_PARSER_SIMGEN_LATENCY:
            simgen_udp_latency_log_parser(input_binary);
            break;
        case LOG_PARSER_GPS_RECEVIER:
            gps_recevier_log_parser(input_binary);
            break;
        case LOG_PARSER_DM_NSPO_REFINE:
            rocket_dynamic_nspo_log_parser(input_binary, target_value);
            break;
        default:
            fprintf(stderr, "Unknown parsing_mode...EXIT\n");
            goto EXIT;
    }
EXIT:
    if (input_binary)
        fclose(input_binary);
    return 0;
}
