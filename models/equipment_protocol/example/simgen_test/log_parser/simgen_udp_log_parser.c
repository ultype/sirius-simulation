#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "simgen_udp_cmd.h"
#define CYCLE_TIME  10 //  ms

int main(int argc, char const *argv[]) {
    /* parse UDP binary log file */
    FILE *input_binary, *output_text;
    struct simgen_udp_command_t udp_cmd;
    int iter;
    double tx_time;

    if ((input_binary = fopen("UDP_mot.bin", "rb")) == NULL) {

        return EXIT_FAILURE;
    }
    if ((output_text = fopen("UDP_mot.txt", "w+")) == NULL) {
        fclose(input_binary);
        return EXIT_FAILURE;
    }
    for(iter = 0 ; ; iter++) {
        if (sizeof(udp_cmd) != fread(&udp_cmd, 1, sizeof(udp_cmd), input_binary))
            break ;

        if (udp_cmd.time_of_validity_ms_ == 0) {
            iter = 0 ;
            fprintf(output_text, "***New Run***\n");
        }
        if (iter >= 2)
            tx_time = (iter - 2) * CYCLE_TIME;
        else
            tx_time = 0;

        fprintf(output_text,"%4d: tx_time_ms = %10.3f  sim_time_ms = %6d    latency = %d\n", iter,
                tx_time, udp_cmd.time_of_validity_ms_, udp_cmd.latency_wrt_tov_and_current_tir_ms_);
    }
    fclose(input_binary);
    fclose(output_text);
    return 0;
}

