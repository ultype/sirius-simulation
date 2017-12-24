#ifndef MODELS_ICF_INCLUDE_ICF_DRIVERS_H
#define MODELS_ICF_INCLUDE_ICF_DRIVERS_H


struct icf_driver_ops {
    /** Name of the driver interface */
    const char *name;
    /** One line description of the driver interface */
    const char *desc;
    int (*open_interface)(void *priv_data, const char *ifname);
    int (*recv_data)(void *priv_data);
    int (*send_data)(void *priv_data);
    int (*send_directly)(void *priv_data);
    int (*close_interface)(void *priv_data);
};

extern const struct icf_driver_ops *const icf_drivers[];

#endif // MODELS_ICF_INCLUDE_ICF_DRIVERS_H
