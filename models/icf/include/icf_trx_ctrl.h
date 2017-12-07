#ifndef MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
#define MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      TRX Ctrl
LIBRARY DEPENDENCY:
      (
      	(../src/icf_utility.c)
      	(../src/socket_can.c)
      	(../src/icf_trx_ctrl.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include "socket_can.h"
#include "icf_utility.h"

struct icf_rx_ctrl_t {
    struct can_device_info_t   can_info;
};

#ifdef __cplusplus
extern "C" {
#endif


int icf_rx_ctrl_job(struct icf_rx_ctrl_t* C);

#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
