#include "icf_drivers.h"

extern struct icf_driver_ops icf_driver_socketcan_ops;
const struct icf_driver_ops *const icf_drivers[] =
{
    &icf_driver_socketcan_ops,
    NULL
};