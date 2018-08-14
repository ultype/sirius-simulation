#ifndef __TRANSMIT_CHANNEL_HH
#define __TRANSMIT_CHANNEL_HH

typedef struct __attribute__((packed)) {
    int prn;
    double pos[3];
    double vel[3];
    double range;
    double clk[2];
} transmit_channel;

#endif
