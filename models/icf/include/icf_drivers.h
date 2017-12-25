#ifndef MODELS_ICF_INCLUDE_ICF_DRIVERS_H
#define MODELS_ICF_INCLUDE_ICF_DRIVERS_H
#include "icf_export.h"
struct icf_driver_ops {
    int (*open_interface)(void **priv_data, char *ifname);
    int (*recv_data)(void *priv_data, uint8_t *rx_buff, uint32_t buff_size);
    int (*send_data)(void *priv_data);
    int (*send_directly)(void *priv_data);

    int  (*select)(void *priv_data, struct timeval *timeout);
    void (*fd_clr)(void *priv_data);
    int  (*fd_isset)(void *priv_data);
    void (*fd_set)(void *priv_data);
    void (*fd_zero)(void *priv_data);

    int (*close_interface)(void *priv_data);
};

extern struct icf_driver_ops *icf_drivers[];

#endif // MODELS_ICF_INCLUDE_ICF_DRIVERS_H
