#include "icf_drivers.h"

extern struct icf_driver_ops icf_driver_socketcan_ops;
extern struct icf_driver_ops icf_driver_rs422_ops;
extern struct icf_driver_ops icf_driver_ethernet_ops;
struct icf_driver_ops *icf_drivers[] = {
    &icf_driver_socketcan_ops,
    &icf_driver_rs422_ops,
    &icf_driver_ethernet_ops,
    NULL
};
