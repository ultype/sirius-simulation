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
        (../src/rs422_serialport.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include "socket_can.h"
#include "icf_utility.h"
#include "ringbuffer.h"
#include "rs422_serialport.h"

struct icf_rx_ctrl_t {
    struct can_device_info_t   can_info;
    struct ringbuffer_t g_tvc_ring;
};

struct icf_tx_ctrl_t {
    struct rs422_device_info_t rs422_info[RS422_TXQ_NUM];
    struct ringbuffer_t icf_tx_ring[RS422_TXQ_NUM];

};

#ifdef __cplusplus
extern "C" {
#endif

int icf_rx_ctrl_init(struct icf_rx_ctrl_t* C);
int icf_rx_ctrl_deinit(struct icf_rx_ctrl_t* C);
int icf_rx_ctrl_job(struct icf_rx_ctrl_t* C);

int icf_tx_direct(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size);
int icf_tx_send2ring(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size);
int icf_tx_ctrl_job(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx);
void *icf_alloc_mem(size_t size);
void icf_free_mem(void *ptr);

int icf_tx_ctrl_init(struct icf_tx_ctrl_t* C);
int icf_tx_ctrl_deinit(struct icf_tx_ctrl_t* C);

#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
