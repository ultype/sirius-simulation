#ifndef MODELS_ICF_INCLUDE_ICF_DRIVERS_H
#define MODELS_ICF_INCLUDE_ICF_DRIVERS_H

typedef enum _ENUM_INTERFACE_TYPE {
    INTERFACE_EMPTY = 0,
    INTERFACE_CANBUS = 1,
    INTERFACE_RS422 = 2,
    INTERFACE_ETHERNET = 3,
    INTERFACE_NUM
}ENUM_INTERFACE_TYPE;

typedef enum _ENUM_INTERFACE_DIR {
    TX = 0,
    RX = 1
}ENUM_INTERFACE_DIR;

struct icf_interface_params {
    char inf_name[IFNAMSIZ];
    int inf_fd;
    ENUM_INTERFACE_TYPE inf_type;
    ENUM_INTERFACE_DIR direction;
    uint32_t qidx_per_interface;
    struct ringbuffer_t icf_ring;
    void *priv_info;
};

struct icf_interface_ops {
    /** Name of the driver interface */
    const char *name;
    /** One line description of the driver interface */
    const char *desc;
    int (*init_interface)(void *priv);
    int (*open_interface)(void *priv);
    int (*recv)(void *priv);
    int (*send_directly)(void *priv);
    int (*enqueue2ring)(void *priv);
    int (*close_interface)(void *priv);
};

#endif // MODELS_ICF_INCLUDE_ICF_DRIVERS_H
