#include "simgen_remote.h"

char *nRU = "RU\n";
char *nTR = "TR, 0\n";
char *nAR = "AR\n";
char *nSET_TIOP = "SET_TIOP,0,2,GATED,1PPS\n";
char *nEN = "-,EN,1\n";

/* struct UDP_Commnad is defined in SimGen_UDP_rx_commands.h*/
struct UDP_command udp_mot;

/* init MOT */
char *t0;
char tn[512];
char tcp_ack_buff[512];

int simgen_init(void) {
    int ret;
    char *tcp_cmd
}

