#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "simgen_udp_cmd.h"

int main(int argc, char const *argv[]) {
    /* parse UDP binary log file */
    FILE *input_binary, *output_text;
    struct simgen_udp_command_t udp_cmd;
    int iter;
    char file_name[1024] = {};
    strcpy(file_name, argv[1]);
    if ((input_binary = fopen(file_name, "rb")) == NULL) {
        fprintf(stderr, "[ERROR] The file name does not exist\n");
        return EXIT_FAILURE;
    }
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
    fclose(input_binary);
    fclose(output_text);
    return 0;
}

