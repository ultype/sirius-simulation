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
        (../src/ringbuffer.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include "socket_can.h"
#include "icf_utility.h"
#include "ringbuffer.h"

struct icf_rx_ctrl_t {
    struct can_device_info_t   can_info;
    struct ringbuffer_t g_tvc_ring;
};

#ifdef __cplusplus
extern "C" {
#endif

int icf_rx_ctrl_init(struct icf_rx_ctrl_t* C);
int icf_rx_ctrl_deinit(struct icf_rx_ctrl_t* C);
int icf_rx_ctrl_job(struct icf_rx_ctrl_t* C);
void *icf_alloc_mem(size_t size);
void icf_free_mem(void *ptr);

#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
